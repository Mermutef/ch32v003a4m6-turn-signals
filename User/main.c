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

// Пины управления поворотниками
#define TURN_LEFT_IN_PIN      GPIO_Pin_4    // PC4: Вход левого поворотника
#define TURN_LEFT_IN_PORT     GPIOC
#define TURN_RIGHT_IN_PIN     GPIO_Pin_6    // PC6: Вход правого поворотника
#define TURN_RIGHT_IN_PORT    GPIOC
#define TURN_LEFT_OUT_PIN     GPIO_Pin_7    // PC7: Выход левого поворотника
#define TURN_LEFT_OUT_PORT    GPIOC
#define TURN_RIGHT_OUT_PIN    GPIO_Pin_6    // PD6: Выход правого поворотника
#define TURN_RIGHT_OUT_PORT   GPIOD

// Параметры системы блокировки
#define SPEED_INT_MIN_INTERVAL   1000       // Максимальный интервал между импульсами для валидного измерения (мс)
#define SPEED_INT_THRESHOLD      142        // Порог импульсов для активации блокировки
#define DOOR_LOCK_PULSE_MS       300        // Длительность импульса блокировки (мс)

// Константы для работы поворотников
#define TURN_SHORT_PRESS_MS    200          // Максимальное время короткого нажатия (мс)
#define TURN_COUNT             5            // Количество миганий для короткого нажатия
#define TURN_TIME_MS           250          // Длительность одного мигания (мс)
#define TURN_ACTIVATION_MS     (TURN_COUNT * TURN_TIME_MS) // Время активации поворотника

// Константы для работы аварийки
#define HAZARD_SHORT_PRESS_MS  200          // Максимальное время короткого нажатия (мс)
#define HAZARD_COUNT           3            // Количество миганий для короткого нажатия
#define HAZARD_TIME_MS         250          // Длительность одного мигания (мс)
#define HAZARD_ACTIVATION_MS   (HAZARD_COUNT * HAZARD_TIME_MS) // Время активации аварийки
#define HAZARD_PRESS_DELAY_MS  25           // Максимальная задержка между нажатиями для детектирования аварийки

// Состояния для умных поворотников
#define TURN_STATE_IDLE        0
#define TURN_STATE_ACTIVE      1

// Состояния для аварийки
#define HAZARD_STATE_IDLE      0
#define HAZARD_STATE_ACTIVE    1

// Структура для управления поворотниками
typedef struct {
    uint8_t state;
    uint32_t press_start_time;
    uint32_t activation_start_time;
} turn_signal_t;

// ========================================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// ========================================================

// Состояние системы омывателя
volatile uint8_t washer_button_pressed = 0;
volatile uint8_t valve_should_activate = 0;
volatile uint8_t wash_cycle_count = 0;

// Состояние системы блокировки
volatile uint32_t system_time_ms = 0;
volatile uint32_t last_speed_int_time = 0;
volatile uint16_t speed_int_count = 0;
volatile uint8_t door_lock_active = 0;
volatile uint8_t door_lock_pulse_active = 0;
volatile uint32_t door_lock_pulse_start = 0;

// Флаги событий для основного цикла
volatile uint8_t exti0_event = 0;
volatile uint8_t exti1_event = 0;
volatile uint8_t exti2_event = 0;
volatile uint8_t exti3_event = 0;
volatile uint8_t exti4_event = 0;
volatile uint8_t exti6_event = 0;

// Состояние поворотников
volatile turn_signal_t left_turn = {0};
volatile turn_signal_t right_turn = {0};
volatile turn_signal_t hazard = {0};

// Флаги для отслеживания нажатий поворотников
volatile uint8_t left_button_pressed = 0;
volatile uint8_t right_button_pressed = 0;
volatile uint32_t left_press_time = 0;
volatile uint32_t right_press_time = 0;

// ========================================================
// ПРОТОТИПЫ ФУНКЦИЙ
// ========================================================

// Функции инициализации системы
void setup_system(void);
void setup_tim(void);
void setup_gpio(void);
void setup_exti(void);

// Функции конфигурации GPIO
void init_washer_gpio_config(void);
void init_door_lock_gpio_config(void);
void init_turn_signals_config(void);
void GPIO_Init_IPU(uint8_t pin, GPIO_TypeDef *port);
void GPIO_Init_IPD(uint8_t pin, GPIO_TypeDef *port);
void GPIO_Init_OPP(uint8_t pin, GPIO_TypeDef *port);

// Функции конфигурации прерываний
void init_washer_exti_config(void);
void init_door_lock_exti_config(void);
void init_turn_signals_exti_config(void);

// Функции управления системами
void set_washer_system_state(uint8_t enabled);
void update_valve_state(void);
void process_speed_sensor(void);
void reset_speed_counter(void);
void set_speed_sensor_state(uint8_t enabled);
void process_turn_signals(void);

// Обработчики прерываний
void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

// ========================================================
// ОСНОВНАЯ ФУНКЦИЯ
// ========================================================
int main(void) {
    // Инициализация системы
    setup_system();
    setup_gpio();
    setup_exti();
    set_washer_system_state(GPIO_ReadInputDataBit(WASHER_ENABLE_PORT, WASHER_ENABLE_PIN) == Bit_SET);

    // Основной цикл
    while (1) {
        __asm__ volatile ("wfi");

        // Обработка события включения/выключения омывателя
        if (exti0_event) {
            exti0_event = 0;
            BitAction state = GPIO_ReadInputDataBit(WASHER_ENABLE_PORT, WASHER_ENABLE_PIN);
            set_washer_system_state(state == Bit_SET);
        }

        // Обработка события нажатия кнопки омывателя
        if (exti1_event) {
            exti1_event = 0;
            BitAction state = GPIO_ReadInputDataBit(WASHER_BUTTON_PORT, WASHER_BUTTON_PIN);
            if (state == Bit_RESET) {
                update_valve_state();
            }
        }

        // Обработка события датчика скорости
        if (exti2_event) {
            exti2_event = 0;
            BitAction state = GPIO_ReadInputDataBit(SPEED_SENSOR_PORT, SPEED_SENSOR_PIN);
            if (state == Bit_RESET) {
                process_speed_sensor();
            }
        }

        // Обработка события открытия двери
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

        // Обработка левого поворотника
        if (exti4_event) {
            exti4_event = 0;
            BitAction state = GPIO_ReadInputDataBit(TURN_LEFT_IN_PORT, TURN_LEFT_IN_PIN);

            if (state == Bit_SET) {
                // Начало нажатия - запоминаем время
                left_turn.press_start_time = system_time_ms;
                left_button_pressed = 1;
                left_press_time = system_time_ms;

                // Проверяем, не нажата ли правая кнопка (аварийка)
                if (right_button_pressed && (system_time_ms - right_press_time < HAZARD_PRESS_DELAY_MS)) {
                    // Активируем аварийку
                    hazard.state = HAZARD_STATE_ACTIVE;
                    hazard.activation_start_time = system_time_ms;

                    // Включаем оба поворотника
                    GPIO_ResetBits(TURN_LEFT_OUT_PORT, TURN_LEFT_OUT_PIN);
                    GPIO_ResetBits(TURN_RIGHT_OUT_PORT, TURN_RIGHT_OUT_PIN);

                    // Сбрасываем состояния поворотников
                    left_turn.state = TURN_STATE_IDLE;
                    right_turn.state = TURN_STATE_IDLE;
                }
            } else {
                // Конец нажатия
                left_button_pressed = 0;

                // Проверяем, была ли активирована аварийка
                if (hazard.state != HAZARD_STATE_ACTIVE) {
                    // Проверяем, было ли это короткое нажатие
                    if (system_time_ms - left_turn.press_start_time >= TURN_SHORT_PRESS_MS) {
                        // Короткое нажатие - активируем на фиксированное время

                        // Прерываем противоположный поворотник
                        if (right_turn.state == TURN_STATE_ACTIVE) {
                            right_turn.state = TURN_STATE_IDLE;
                            GPIO_SetBits(TURN_RIGHT_OUT_PORT, TURN_RIGHT_OUT_PIN);
                        }

                        // Активируем текущий поворотник
                        left_turn.state = TURN_STATE_ACTIVE;
                        left_turn.activation_start_time = system_time_ms;
                        GPIO_ResetBits(TURN_LEFT_OUT_PORT, TURN_LEFT_OUT_PIN);
                    }
                }
            }
        }

        // Обработка правого поворотника
        if (exti6_event) {
            exti6_event = 0;
            BitAction state = GPIO_ReadInputDataBit(TURN_RIGHT_IN_PORT, TURN_RIGHT_IN_PIN);

            if (state == Bit_SET) {
                // Начало нажатия - запоминаем время
                right_turn.press_start_time = system_time_ms;
                right_button_pressed = 1;
                right_press_time = system_time_ms;

                // Проверяем, не нажата ли левая кнопка (аварийка)
                if (left_button_pressed && (system_time_ms - left_press_time < HAZARD_PRESS_DELAY_MS)) {
                    // Активируем аварийку
                    hazard.state = HAZARD_STATE_ACTIVE;
                    hazard.activation_start_time = system_time_ms;

                    // Включаем оба поворотника
                    GPIO_ResetBits(TURN_LEFT_OUT_PORT, TURN_LEFT_OUT_PIN);
                    GPIO_ResetBits(TURN_RIGHT_OUT_PORT, TURN_RIGHT_OUT_PIN);

                    // Сбрасываем состояния поворотников
                    left_turn.state = TURN_STATE_IDLE;
                    right_turn.state = TURN_STATE_IDLE;
                }
            } else {
                // Конец нажатия
                right_button_pressed = 0;

                // Проверяем, была ли активирована аварийка
                if (hazard.state != HAZARD_STATE_ACTIVE) {
                    // Проверяем, было ли это короткое нажатие
                    if (system_time_ms - right_turn.press_start_time >= TURN_SHORT_PRESS_MS) {
                        // Короткое нажатие - активируем на фиксированное время

                        // Прерываем противоположный поворотник
                        if (left_turn.state == TURN_STATE_ACTIVE) {
                            left_turn.state = TURN_STATE_IDLE;
                            GPIO_SetBits(TURN_LEFT_OUT_PORT, TURN_LEFT_OUT_PIN);
                        }

                        // Активируем текущий поворотник
                        right_turn.state = TURN_STATE_ACTIVE;
                        right_turn.activation_start_time = system_time_ms;
                        GPIO_ResetBits(TURN_RIGHT_OUT_PORT, TURN_RIGHT_OUT_PIN);
                    }
                }
            }
        }

        // Обработка импульса блокировки дверей
        if (door_lock_pulse_active) {
            if (system_time_ms - door_lock_pulse_start >= DOOR_LOCK_PULSE_MS) {
                GPIO_ResetBits(LOCK_DOOR_PORT, LOCK_DOOR_PIN);
                door_lock_pulse_active = 0;
            }
        }

        // Обработка времени активации поворотников
        process_turn_signals();
    }
}

// ========================================================
// ФУНКЦИИ ИНИЦИАЛИЗАЦИИ СИСТЕМЫ
// ========================================================

// Инициализация системы
void setup_system(void) {
    SystemInit();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    Delay_Init();
    setup_tim();
}

// Настройка таймера
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

// Настройка GPIO
void setup_gpio(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    init_washer_gpio_config();
    init_door_lock_gpio_config();
    init_turn_signals_config();
}

// Конфигурация GPIO для системы омывателя
void init_washer_gpio_config(void) {
    GPIO_Init_IPU(WASHER_ENABLE_PIN, WASHER_ENABLE_PORT);
    GPIO_Init_IPD(WASHER_BUTTON_PIN, WASHER_BUTTON_PORT);
    GPIO_Init_OPP(VALVE_CONTROL_PIN, VALVE_CONTROL_PORT);
    GPIO_ResetBits(VALVE_CONTROL_PORT, VALVE_CONTROL_PIN);
}

// Конфигурация GPIO для системы центрального замка
void init_door_lock_gpio_config(void) {
    GPIO_Init_IPD(SPEED_SENSOR_PIN, SPEED_SENSOR_PORT);
    GPIO_Init_IPU(UNLOCK_DOOR_PIN, UNLOCK_DOOR_PORT);
    GPIO_Init_OPP(LOCK_DOOR_PIN, LOCK_DOOR_PORT);
}

// Конфигурация GPIO для системы поворотников
void init_turn_signals_config(void) {
    GPIO_Init_IPD(TURN_LEFT_IN_PIN, TURN_LEFT_IN_PORT);
    GPIO_Init_IPD(TURN_RIGHT_IN_PIN, TURN_RIGHT_IN_PORT);

    GPIO_Init_OPP(TURN_LEFT_OUT_PIN, TURN_LEFT_OUT_PORT);
    GPIO_Init_OPP(TURN_RIGHT_OUT_PIN, TURN_RIGHT_OUT_PORT);

    GPIO_SetBits(TURN_LEFT_OUT_PORT, TURN_LEFT_OUT_PIN);
    GPIO_SetBits(TURN_RIGHT_OUT_PORT, TURN_RIGHT_OUT_PIN);
}

// Инициализация входа с подтяжкой к питанию
void GPIO_Init_IPU(const uint8_t pin, GPIO_TypeDef *port) {
    GPIO_InitTypeDef gpio_init;
    gpio_init.GPIO_Pin = pin;
    gpio_init.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(port, &gpio_init);
}

// Инициализация входа с подтяжкой к земле
void GPIO_Init_IPD(const uint8_t pin, GPIO_TypeDef *port) {
    GPIO_InitTypeDef gpio_init;
    gpio_init.GPIO_Pin = pin;
    gpio_init.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(port, &gpio_init);
}

// Инициализация выхода с push-pull конфигурацией
void GPIO_Init_OPP(const uint8_t pin, GPIO_TypeDef *port) {
    GPIO_InitTypeDef gpio_init;
    gpio_init.GPIO_Pin = pin;
    gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(port, &gpio_init);
}

// ========================================================
// КОНФИГУРАЦИЯ ПРЕРЫВАНИЙ
// ========================================================

// Настройка прерываний
void setup_exti(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    init_washer_exti_config();
    init_door_lock_exti_config();
    init_turn_signals_exti_config();

    NVIC_InitTypeDef nvic_init;
    nvic_init.NVIC_IRQChannel = EXTI7_0_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
    nvic_init.NVIC_IRQChannelSubPriority = 0;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
}

// Конфигурация прерываний для системы омывателя
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

// Конфигурация прерываний для системы центрального замка
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

// Конфигурация прерываний для системы поворотников
void init_turn_signals_exti_config(void) {
    EXTI_InitTypeDef exti_init;

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
    exti_init.EXTI_Line = EXTI_Line4;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    exti_init.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource6);
    exti_init.EXTI_Line = EXTI_Line6;
    EXTI_Init(&exti_init);
}

// ========================================================
// ОБРАБОТЧИКИ ПРЕРЫВАНИЙ
// ========================================================

// Обработчик прерываний EXTI линии 0-7
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

    if (EXTI_GetITStatus(EXTI_Line4) != RESET) {
        exti4_event = 1;
        EXTI_ClearITPendingBit(EXTI_Line4);
    }

    if (EXTI_GetITStatus(EXTI_Line6) != RESET) {
        exti6_event = 1;
        EXTI_ClearITPendingBit(EXTI_Line6);
    }
}

// Обработчик прерываний таймера TIM1
void TIM1_UP_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_Update)) {
        ++system_time_ms;
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

// ========================================================
// ФУНКЦИИ УПРАВЛЕНИЯ СИСТЕМАМИ
// ========================================================

// Установка состояния системы омывателя
void set_washer_system_state(const uint8_t enabled) {
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

// Обновление состояния клапана омывателя
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

// Обработка сигнала датчика скорости
void process_speed_sensor(void) {
    if (!door_lock_active) {
        uint32_t time_since_last = system_time_ms - last_speed_int_time;

        if (time_since_last < SPEED_INT_MIN_INTERVAL) {
            ++speed_int_count;
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

// Сброс счетчика скорости
void reset_speed_counter(void) {
    last_speed_int_time = system_time_ms;
    speed_int_count = 0;
}

// Установка состояния датчика скорости
void set_speed_sensor_state(const uint8_t enabled) {
    EXTI_InitTypeDef exti_init;
    exti_init.EXTI_Line = EXTI_Line2;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
    exti_init.EXTI_LineCmd = enabled ? ENABLE : DISABLE;
    EXTI_Init(&exti_init);
}

// Обработка времени активации поворотников и аварийки
void process_turn_signals(void) {
    // Обработка левого поворотника
    if (left_turn.state == TURN_STATE_ACTIVE) {
        // Проверяем, не истекло ли время активации
        if (system_time_ms - left_turn.activation_start_time >= TURN_ACTIVATION_MS) {
            left_turn.state = TURN_STATE_IDLE;
            GPIO_SetBits(TURN_LEFT_OUT_PORT, TURN_LEFT_OUT_PIN); // Снимаем массу
        }
    }

    // Обработка правого поворотника
    if (right_turn.state == TURN_STATE_ACTIVE) {
        if (system_time_ms - right_turn.activation_start_time >= TURN_ACTIVATION_MS) {
            right_turn.state = TURN_STATE_IDLE;
            GPIO_SetBits(TURN_RIGHT_OUT_PORT, TURN_RIGHT_OUT_PIN); // Снимаем массу
        }
    }

    // Обработка аварийки
    if (hazard.state == HAZARD_STATE_ACTIVE) {
        if (system_time_ms - hazard.activation_start_time >= HAZARD_ACTIVATION_MS) {
            hazard.state = HAZARD_STATE_IDLE;
            GPIO_SetBits(TURN_LEFT_OUT_PORT, TURN_LEFT_OUT_PIN); // Снимаем массу
            GPIO_SetBits(TURN_RIGHT_OUT_PORT, TURN_RIGHT_OUT_PIN); // Снимаем массу
        }
    }
}