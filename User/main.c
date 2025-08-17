#include "ch32v00x_gpio.h"
#include "ch32v00x_rcc.h"
#include "ch32v00x_exti.h"
#include "ch32v00x_misc.h"

// ========================================================
// КОНФИГУРАЦИЯ ОБОРУДОВАНИЯ
// ========================================================

// Пины управления системой омывателя фар
#define WASHER_ENABLE_PIN     GPIO_Pin_0    // PC0: Включение/выключение омывателя
#define WASHER_ENABLE_PORT    GPIOC
#define WASHER_BUTTON_PIN     GPIO_Pin_1    // PC1: Кнопка активации омывателя
#define WASHER_BUTTON_PORT    GPIOC
#define VALVE_CONTROL_PIN     GPIO_Pin_4    // PD4: Управление клапаном омывателя
#define VALVE_CONTROL_PORT    GPIOD

// Пины управления центральным замком
#define SPEED_SENSOR_PIN      GPIO_Pin_2    // PC2: Сигнал с датчика скорости
#define SPEED_SENSOR_PORT     GPIOC
#define UNLOCK_DOOR_PIN       GPIO_Pin_3    // PC3: Сигнал открытия центрального замка
#define UNLOCK_DOOR_PORT      GPIOC
#define LOCK_DOOR_PIN         GPIO_Pin_5    // PD5: Закрытие центрального замка
#define LOCK_DOOR_PORT        GPIOD

// Параметры системы блокировки
#define SPEED_INT_MIN_INTERVAL   1000       // Максимальный интервал между импульсами для валидного измерения (мс)
#define SPEED_INT_THRESHOLD      142        // Порог импульсов для активации блокировки
#define DOOR_LOCK_PULSE_MS       300        // Длительность импульса блокировки (мс)

// Глобальные переменные состояния системы
volatile uint8_t washer_button_pressed = 0; // Флаг нажатия кнопки омывателя
volatile uint8_t valve_should_activate = 0; // Флаг активации клапана
volatile uint8_t wash_cycle_count = 0; // Счетчик циклов омывателя
volatile uint32_t system_time_ms = 0; // Системное время (мс)
volatile uint32_t last_speed_int_time = 0; // Время последнего импульса датчика скорости
volatile uint16_t speed_int_count = 0; // Счетчик импульсов датчика скорости
volatile uint8_t door_lock_active = 0; // Флаг активности центрального замка

// Флаги событий для основного цикла
volatile uint8_t exti0_event = 0;
volatile uint8_t exti1_event = 0;
volatile uint8_t exti2_event = 0;
volatile uint8_t exti3_event = 0;

// Состояние импульса блокировки
volatile uint8_t door_lock_pulse_active = 0;
volatile uint32_t door_lock_pulse_start = 0;

// ========================================================
// ПРОТОТИПЫ ФУНКЦИЙ
// ========================================================

// Функции инициализации системы
void setup_system(void);

void setup_tim(void);

void setup_gpio(void);

void setup_exti(void);

// Конфигурация GPIO
void init_washer_gpio_config(void);

void init_door_lock_gpio_config(void);

void GPIO_Init_IPU(uint8_t pin, GPIO_TypeDef *port);

void GPIO_Init_IPD(uint8_t pin, GPIO_TypeDef *port);

void GPIO_Init_OPP(uint8_t pin, GPIO_TypeDef *port);

// Конфигурация прерываний
void init_washer_exti_config(void);

void init_door_lock_exti_config(void);

// Управление системами
void set_washer_system_state(uint8_t enabled);

void update_valve_state(void);

void process_speed_sensor(void);

void reset_speed_counter(void);

void set_speed_sensor_state(uint8_t enabled);

// Обработчики прерываний
void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

// ========================================================
// ОСНОВНАЯ ФУНКЦИЯ
// ========================================================
int main(void) {
    // Инициализация системных компонентов
    setup_system();

    // Настройка периферии
    setup_gpio();
    setup_exti();

    // Инициализация начального состояния омывателя
    set_washer_system_state(GPIO_ReadInputDataBit(WASHER_ENABLE_PORT, WASHER_ENABLE_PIN) == Bit_SET);

    // Основной цикл работы
    while (1) {
        __asm__ volatile ("wfi"); // Ожидание прерывания

        // Обработка событий прерываний
        if (exti0_event) {
            exti0_event = 0;
            BitAction state = GPIO_ReadInputDataBit(WASHER_ENABLE_PORT, WASHER_ENABLE_PIN);
            set_washer_system_state(state == Bit_SET);
        }

        if (exti1_event) {
            exti1_event = 0;
            BitAction state = GPIO_ReadInputDataBit(WASHER_BUTTON_PORT, WASHER_BUTTON_PIN);
            if (state == Bit_RESET) {
                update_valve_state();
            }
        }

        if (exti2_event) {
            exti2_event = 0;
            BitAction state = GPIO_ReadInputDataBit(SPEED_SENSOR_PORT, SPEED_SENSOR_PIN);
            if (state == Bit_RESET) {
                process_speed_sensor();
            }
        }

        if (exti3_event) {
            exti3_event = 0;
            BitAction state = GPIO_ReadInputDataBit(UNLOCK_DOOR_PORT, UNLOCK_DOOR_PIN);
            if (state == Bit_RESET) {
                GPIO_ResetBits(LOCK_DOOR_PORT, LOCK_DOOR_PIN);
                door_lock_active = 0;
                reset_speed_counter();
                set_speed_sensor_state(1);
            }
        }

        // Управление длительностью импульса блокировки
        if (door_lock_pulse_active) {
            if (system_time_ms - door_lock_pulse_start >= DOOR_LOCK_PULSE_MS) {
                GPIO_ResetBits(LOCK_DOOR_PORT, LOCK_DOOR_PIN);
                door_lock_pulse_active = 0;
            }
        }
    }
}

// ========================================================
// ФУНКЦИИ ИНИЦИАЛИЗАЦИИ СИСТЕМЫ
// ========================================================
void setup_system(void) {
    SystemInit();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    Delay_Init();
    setup_tim();
}

void setup_tim(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitTypeDef tim_cfg;
    tim_cfg.TIM_Period = SystemCoreClock / 1000;
    tim_cfg.TIM_Prescaler = 0;
    tim_cfg.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_cfg.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &tim_cfg);

    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    NVIC_InitTypeDef nvic_cfg;
    nvic_cfg.NVIC_IRQChannel = TIM1_UP_IRQn;
    nvic_cfg.NVIC_IRQChannelPreemptionPriority = 0;
    nvic_cfg.NVIC_IRQChannelSubPriority = 0;
    nvic_cfg.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_cfg);
}

// ========================================================
// КОНФИГУРАЦИЯ GPIO
// ========================================================
void setup_gpio(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    init_washer_gpio_config();
    init_door_lock_gpio_config();
}

void init_washer_gpio_config(void) {
    GPIO_Init_IPU(WASHER_ENABLE_PIN, WASHER_ENABLE_PORT);
    GPIO_Init_IPD(WASHER_BUTTON_PIN, WASHER_BUTTON_PORT);
    GPIO_Init_OPP(VALVE_CONTROL_PIN, VALVE_CONTROL_PORT);
    GPIO_ResetBits(VALVE_CONTROL_PORT, VALVE_CONTROL_PIN);
}

void init_door_lock_gpio_config(void) {
    GPIO_Init_IPD(SPEED_SENSOR_PIN, SPEED_SENSOR_PORT);
    GPIO_Init_IPU(UNLOCK_DOOR_PIN, UNLOCK_DOOR_PORT);
    GPIO_Init_OPP(LOCK_DOOR_PIN, LOCK_DOOR_PORT);
}

void GPIO_Init_IPU(uint8_t pin, GPIO_TypeDef *port) {
    GPIO_InitTypeDef gpio_init;
    gpio_init.GPIO_Pin = pin;
    gpio_init.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(port, &gpio_init);
}

void GPIO_Init_IPD(uint8_t pin, GPIO_TypeDef *port) {
    GPIO_InitTypeDef gpio_init;
    gpio_init.GPIO_Pin = pin;
    gpio_init.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(port, &gpio_init);
}

void GPIO_Init_OPP(uint8_t pin, GPIO_TypeDef *port) {
    GPIO_InitTypeDef gpio_init;
    gpio_init.GPIO_Pin = pin;
    gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(port, &gpio_init);
}

// ========================================================
// КОНФИГУРАЦИЯ ПРЕРЫВАНИЙ
// ========================================================
void setup_exti(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    init_washer_exti_config();
    init_door_lock_exti_config();

    NVIC_InitTypeDef nvic_init;
    nvic_init.NVIC_IRQChannel = EXTI7_0_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
    nvic_init.NVIC_IRQChannelSubPriority = 0;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
}

void init_washer_exti_config(void) {
    EXTI_InitTypeDef exti_init;

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);
    exti_init.EXTI_Line = EXTI_Line0;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    exti_init.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1);
    exti_init.EXTI_Line = EXTI_Line1;
    exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_Init(&exti_init);
}

void init_door_lock_exti_config(void) {
    EXTI_InitTypeDef exti_init;

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);
    exti_init.EXTI_Line = EXTI_Line2;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
    exti_init.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource3);
    exti_init.EXTI_Line = EXTI_Line3;
    EXTI_Init(&exti_init);
}

// ========================================================
// ОБРАБОТЧИКИ ПРЕРЫВАНИЙ
// ========================================================
void EXTI7_0_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        exti0_event = 1;
        EXTI_ClearITPendingBit(EXTI_Line0);
    }

    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        exti1_event = 1;
        EXTI_ClearITPendingBit(EXTI_Line1);
    }

    if (EXTI_GetITStatus(EXTI_Line2) != RESET) {
        exti2_event = 1;
        EXTI_ClearITPendingBit(EXTI_Line2);
    }

    if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
        exti3_event = 1;
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}

void TIM1_UP_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_Update)) {
        system_time_ms++; // Простой инкремент вместо атомарной операции
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

// ========================================================
// ФУНКЦИИ УПРАВЛЕНИЯ СИСТЕМАМИ
// ========================================================
void set_washer_system_state(uint8_t enabled) {
    EXTI_InitTypeDef exti_init;
    exti_init.EXTI_Line = EXTI_Line1;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
    exti_init.EXTI_LineCmd = enabled ? ENABLE : DISABLE;
    EXTI_Init(&exti_init);

    if (!enabled) {
        washer_button_pressed = 0;
        valve_should_activate = 0;
        wash_cycle_count = 0;
        GPIO_ResetBits(VALVE_CONTROL_PORT, VALVE_CONTROL_PIN);
    }
}

void update_valve_state(void) {
    wash_cycle_count = (wash_cycle_count + 1) % 3;

    if (wash_cycle_count == 2) {
        valve_should_activate = 1;
    }

    if (valve_should_activate) {
        GPIO_SetBits(VALVE_CONTROL_PORT, VALVE_CONTROL_PIN);
        valve_should_activate = 0;
    }

    if (wash_cycle_count == 0) {
        GPIO_ResetBits(VALVE_CONTROL_PORT, VALVE_CONTROL_PIN);
    }
}

void process_speed_sensor(void) {
    if (!door_lock_active) {
        uint32_t time_since_last = system_time_ms - last_speed_int_time;

        if (time_since_last < SPEED_INT_MIN_INTERVAL) {
            speed_int_count++;
        } else {
            reset_speed_counter();
        }

        if (speed_int_count > SPEED_INT_THRESHOLD) {
            GPIO_SetBits(LOCK_DOOR_PORT, LOCK_DOOR_PIN);
            door_lock_active = 1;
            door_lock_pulse_active = 1;
            door_lock_pulse_start = system_time_ms;
            speed_int_count = 0;
        }
    }
}

void reset_speed_counter(void) {
    last_speed_int_time = system_time_ms;
    speed_int_count = 0;
}

void set_speed_sensor_state(uint8_t enabled) {
    EXTI_InitTypeDef exti_init;
    exti_init.EXTI_Line = EXTI_Line2;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
    exti_init.EXTI_LineCmd = enabled ? ENABLE : DISABLE;
    EXTI_Init(&exti_init);
}
