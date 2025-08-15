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
#define LOCK_RESET_THRESHOLD     10         // Порог импульсов для сброса блокировки

// Глобальные переменные состояния системы
volatile uint8_t washer_button_pressed = 0; // Флаг нажатия кнопки омывателя
volatile uint8_t valve_should_activate = 0; // Флаг активации клапана
volatile uint8_t wash_cycle_count = 0; // Счетчик циклов омывателя
volatile uint64_t system_time_ms = 0; // Системное время
volatile uint64_t last_speed_int_time = 0; // Время последнего прерывания с датчика скорости
volatile uint16_t speed_int_count = 0; // Количество прерываний с датчика скорости
volatile uint8_t door_lock_active = 0; // Флаг закрытия центрального замка

// ========================================================
// ПРОТОТИПЫ ФУНКЦИЙ
// ========================================================

void setup_system();

void setup_tim();

void setup_gpio();

void init_washer_gpio_config();

void init_door_lock_gpio_config();

void GPIO_Init_IPU(uint8_t pin, GPIO_TypeDef *port);

void GPIO_Init_IPD(uint8_t pin, GPIO_TypeDef *port);

void GPIO_Init_OPP(uint8_t pin, GPIO_TypeDef *port);

void setup_exti();

void init_washer_exti_config();

void init_door_lock_exti_config();

void update_valve_state();

void set_washer_system_state(uint8_t enabled);

void process_speed_sensor();

void reset_speed_counter();

void set_speed_sensor_state(uint8_t enabled);

void EXTI7_0_IRQHandler() __attribute__((interrupt("WCH-Interrupt-fast")));

void TIM1_UP_IRQHandler() __attribute__((interrupt("WCH-Interrupt-fast")));


// ========================================================
// ОСНОВНАЯ ФУНКЦИЯ
// ========================================================
int main() {
    // Инициализация системных компонентов
    setup_system();

    // Настройка периферии
    setup_gpio(); // Конфигурация GPIO
    setup_exti(); // Конфигурация прерываний

    // Инициализация начального состояния омывателя
    set_washer_system_state(
        GPIO_ReadInputDataBit(WASHER_ENABLE_PORT, WASHER_ENABLE_PIN) == Bit_RESET
    );

    // Основной цикл работы (режим пониженного энергопотребления)
    while (1) {
        __asm__ volatile ("wfi"); // Ожидание прерывания
    }
}

// ========================================================
// ФУНКЦИИ ИНИЦИАЛИЗАЦИИ
// ========================================================

void setup_system() {
    SystemInit(); // Настройка системных часов
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); // Приоритеты прерываний
    Delay_Init(); // Инициализация функций задержки
    setup_tim(); // Инициализация таймеров
}

void setup_tim() {
    // Включение тактирования таймера
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // Настройка базового таймера для системных тиков
    TIM_TimeBaseInitTypeDef tim_cfg = {
        .TIM_Period = SystemCoreClock / 1000, // 1ms
        .TIM_Prescaler = 0,
        .TIM_ClockDivision = TIM_CKD_DIV1,
        .TIM_CounterMode = TIM_CounterMode_Up
    };
    TIM_TimeBaseInit(TIM1, &tim_cfg);

    // Разрешение прерывания по обновлению
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    // Настройка NVIC для TIM1
    NVIC_InitTypeDef nvic_cfg = {
        .NVIC_IRQChannel = TIM1_UP_IRQn,
        .NVIC_IRQChannelPreemptionPriority = 0,
        .NVIC_IRQChannelSubPriority = 0,
        .NVIC_IRQChannelCmd = ENABLE
    };
    NVIC_Init(&nvic_cfg);
}

// Общая настройка GPIO
void setup_gpio() {
    // Включение тактирования портов
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    init_washer_gpio_config();
    init_door_lock_gpio_config();
}

// Настройка конфигурации GPIO омывателя
void init_washer_gpio_config() {
    // Настройка входа для включения омывателя (PC0)
    GPIO_Init_IPU(WASHER_ENABLE_PIN, WASHER_ENABLE_PORT);

    // Настройка входа для кнопки омывателя (PC1)
    GPIO_Init_IPD(WASHER_BUTTON_PIN, WASHER_BUTTON_PORT);

    // Настройка выхода управления клапаном (PD4)
    GPIO_Init_OPP(VALVE_CONTROL_PIN, VALVE_CONTROL_PORT);

    // Исходное состояние - клапан закрыт
    GPIO_ResetBits(VALVE_CONTROL_PORT, VALVE_CONTROL_PIN);
}

void init_door_lock_gpio_config() {
    // Настройка входа с датчика скорости (PC2)
    GPIO_Init_IPD(SPEED_SENSOR_PIN, SPEED_SENSOR_PORT);

    // Настройка входа открытия центрального замка (PC3)
    GPIO_Init_IPU(UNLOCK_DOOR_PIN, UNLOCK_DOOR_PORT);

    // Настройка выхода закрытия дверей (PD5)
    GPIO_Init_OPP(LOCK_DOOR_PIN, LOCK_DOOR_PORT);
}

void GPIO_Init_IPU(const uint8_t pin, GPIO_TypeDef *port) {
    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.GPIO_Pin = pin;
    gpio_init.GPIO_Mode = GPIO_Mode_IPU; // Вход с подтяжкой к питанию
    GPIO_Init(port, &gpio_init);
}

void GPIO_Init_IPD(const uint8_t pin, GPIO_TypeDef *port) {
    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.GPIO_Pin = pin;
    gpio_init.GPIO_Mode = GPIO_Mode_IPD; // Вход с подтяжкой к земле
    GPIO_Init(port, &gpio_init);
}

void GPIO_Init_OPP(const uint8_t pin, GPIO_TypeDef *port) {
    GPIO_InitTypeDef gpio_init = {0};

    // Настройка выхода закрытия дверей
    gpio_init.GPIO_Pin = pin;
    gpio_init.GPIO_Mode = GPIO_Mode_Out_PP; // Выход push-pull
    gpio_init.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(port, &gpio_init);
}

// Общая настройка прерываний
void setup_exti() {
    // Включение тактирования AFIO (необходимо для EXTI)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    init_washer_exti_config();
    init_door_lock_exti_config();

    // Настройка контроллера прерываний (NVIC)
    NVIC_InitTypeDef nvic_init = {0};
    nvic_init.NVIC_IRQChannel = EXTI7_0_IRQn; // Канал прерываний 0-7
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0; // Приоритет
    nvic_init.NVIC_IRQChannelSubPriority = 0; // Подприоритет
    nvic_init.NVIC_IRQChannelCmd = ENABLE; // Включить канал
    NVIC_Init(&nvic_init);
}

// Настройка конфигурации прерываний омывателя
void init_washer_exti_config() {
    EXTI_InitTypeDef exti_init = {0};

    // Прерывание для кнопки включения омывателя (PC0)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);
    exti_init.EXTI_Line = EXTI_Line0;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // Оба фронта
    exti_init.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init);

    // Прерывание для кнопки омывателя (PC1)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1);
    exti_init.EXTI_Line = EXTI_Line1;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Falling; // Спад сигнала
    exti_init.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init);
}

// Настройка конфигурации прерываний закрытия центрального замка
void init_door_lock_exti_config() {
    EXTI_InitTypeDef exti_init = {0};

    // Прерывание для датчика скорости (PC2)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);
    exti_init.EXTI_Line = EXTI_Line2;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Falling; // Спад сигнала
    exti_init.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init);

    // Прерывание для кнопки открытия центрального замка (PC3)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource3);
    exti_init.EXTI_Line = EXTI_Line3;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Falling; // Спад сигнала
    exti_init.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init);
}

// ========================================================
// ОБРАБОТЧИК ПРЕРЫВАНИЙ
// ========================================================

void EXTI7_0_IRQHandler() {
    // Обработка прерывания от кнопки включения системы (PC0)
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        // Чтение стабильного состояния с защитой от дребезга
        const BitAction state = GPIO_ReadInputDataBit(WASHER_ENABLE_PORT, WASHER_ENABLE_PIN);

        // Обновление состояния системы
        set_washer_system_state(state == Bit_RESET);

        // Сброс флага прерывания
        EXTI_ClearITPendingBit(EXTI_Line0);
    }

    // Обработка прерывания от кнопки омывателя (PC1)
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        // Чтение стабильного состояния с защитой от дребезга
        const BitAction state = GPIO_ReadInputDataBit(WASHER_BUTTON_PORT, WASHER_BUTTON_PIN);

        // Активация клапана при подтвержденном нажатии
        if (state == Bit_RESET) update_valve_state();

        // Сброс флага прерывания
        EXTI_ClearITPendingBit(EXTI_Line1);
    }

    if (EXTI_GetITStatus(EXTI_Line2) != RESET) {
        const BitAction state = GPIO_ReadInputDataBit(SPEED_SENSOR_PORT, SPEED_SENSOR_PIN);

        if (state == Bit_RESET) process_speed_sensor();

        EXTI_ClearITPendingBit(EXTI_Line2);
    }

    if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
        const BitAction state = GPIO_ReadInputDataBit(UNLOCK_DOOR_PORT, UNLOCK_DOOR_PIN);

        if (state == Bit_RESET) {
            GPIO_ResetBits(LOCK_DOOR_PORT, LOCK_DOOR_PIN);
            door_lock_active = 0;
            reset_speed_counter();
            set_speed_sensor_state(1);
        }

        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}

// Обработчик прерывания таймера
void TIM1_UP_IRQHandler() {
    if (TIM_GetITStatus(TIM1, TIM_IT_Update)) {
        ++system_time_ms;
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

// ========================================================
// ФУНКЦИИ УПРАВЛЕНИЯ СИСТЕМОЙ
// ========================================================

// Управление состоянием системы омывателя (включение/выключение)
void set_washer_system_state(const uint8_t enabled) {
    EXTI_InitTypeDef exti_init = {0};

    // Включение/выключение прерывания для кнопки омывателя
    exti_init.EXTI_Line = EXTI_Line1;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
    exti_init.EXTI_LineCmd = enabled ? ENABLE : DISABLE;
    EXTI_Init(&exti_init);

    // Сброс состояния системы при выключении
    if (!enabled) {
        washer_button_pressed = 0;
        valve_should_activate = 0;
        wash_cycle_count = 0;
        GPIO_ResetBits(VALVE_CONTROL_PORT, VALVE_CONTROL_PIN); // Закрыть клапан
    }
}

// Обновление состояния клапана на основе счетчика циклов
void update_valve_state() {
    wash_cycle_count = (wash_cycle_count + 1) % 3; // Цикл 0-1-2

    // Перед третьим нажатием устанавливаем флаг активации
    if (wash_cycle_count == 2) valve_should_activate = 1;

    // Активация клапана при установленном флаге
    if (valve_should_activate) {
        GPIO_SetBits(VALVE_CONTROL_PORT, VALVE_CONTROL_PIN); // Открыть клапан
        valve_should_activate = 0; // Сбросить флаг
    }

    // Закрытие клапана после третьего нажатия
    if (wash_cycle_count == 0) GPIO_ResetBits(VALVE_CONTROL_PORT, VALVE_CONTROL_PIN); // Закрыть клапан
}


void process_speed_sensor() {
    if (!door_lock_active) {
        const uint64_t time_since_last = system_time_ms - last_speed_int_time;

        if (time_since_last < SPEED_INT_MIN_INTERVAL) ++speed_int_count;
        else reset_speed_counter();

        if (speed_int_count > SPEED_INT_THRESHOLD) {
            GPIO_SetBits(LOCK_DOOR_PORT, LOCK_DOOR_PIN);
            door_lock_active = 1;
            speed_int_count = 0;
        }

        return;
    }

    ++speed_int_count;
    if (speed_int_count > LOCK_RESET_THRESHOLD) {
        GPIO_ResetBits(LOCK_DOOR_PORT, LOCK_DOOR_PIN);
        EXTI_InitTypeDef exti_init = {0};

        // Прерывание для датчика скорости (PC2)
        GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);
        exti_init.EXTI_Line = EXTI_Line2;
        exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
        exti_init.EXTI_Trigger = EXTI_Trigger_Falling; // Спад сигнала
        exti_init.EXTI_LineCmd = DISABLE;
        EXTI_Init(&exti_init);
    }
}


void reset_speed_counter() {
    last_speed_int_time = system_time_ms;
    speed_int_count = 0;
}

void set_speed_sensor_state(const uint8_t enabled) {
    EXTI_InitTypeDef exti_init = {0};

    // Включение/выключение прерывания для кнопки омывателя
    exti_init.EXTI_Line = EXTI_Line2;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
    exti_init.EXTI_LineCmd = enabled ? ENABLE : DISABLE;
    EXTI_Init(&exti_init);
}
