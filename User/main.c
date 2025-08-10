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

// Глобальные переменные состояния системы
volatile uint8_t washer_button_pressed = 0; // Флаг нажатия кнопки омывателя
volatile uint8_t valve_should_activate = 0; // Флаг активации клапана
volatile uint8_t wash_cycle_count = 0; // Счетчик циклов омывателя

// ========================================================
// ПРОТОТИПЫ ФУНКЦИЙ
// ========================================================
void setup_gpio(void);

void setup_exti(void);

void init_washer_gpio_config(void);

void init_washer_exti_config(void);

void update_valve_state(void);

static BitAction read_debounced_input(GPIO_TypeDef *port, uint16_t pin, uint8_t samples, uint16_t interval);

void configure_washer_state(uint8_t enabled);

void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

// ========================================================
// ОСНОВНАЯ ФУНКЦИЯ
// ========================================================
int main(void) {
    // Инициализация системных компонентов
    SystemInit(); // Настройка системных часов
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); // Приоритеты прерываний
    Delay_Init(); // Инициализация функций задержки

    // Настройка периферии
    setup_gpio(); // Конфигурация GPIO
    setup_exti(); // Конфигурация прерываний

    // Инициализация начального состояния омывателя
    configure_washer_state(
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

// Общая настройка GPIO
void setup_gpio(void) {
    // Включение тактирования портов
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    init_washer_gpio_config();
}

// Настройка конфигурации GPIO омывателя
void init_washer_gpio_config(void) {
    GPIO_InitTypeDef gpio_init = {0};

    // Настройка входа для включения омывателя (PC0)
    gpio_init.GPIO_Pin = WASHER_ENABLE_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_IPU; // Вход с подтяжкой к питанию
    GPIO_Init(WASHER_ENABLE_PORT, &gpio_init);

    // Настройка входа для кнопки омывателя (PC1)
    gpio_init.GPIO_Pin = WASHER_BUTTON_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_IPD; // Вход с подтяжкой к земле
    GPIO_Init(WASHER_BUTTON_PORT, &gpio_init);

    // Настройка выхода управления клапаном (PD4)
    gpio_init.GPIO_Pin = VALVE_CONTROL_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_Out_PP; // Выход push-pull
    gpio_init.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(VALVE_CONTROL_PORT, &gpio_init);

    // Исходное состояние - клапан закрыт
    GPIO_ResetBits(VALVE_CONTROL_PORT, VALVE_CONTROL_PIN);
}

// Общая настройка прерываний
void setup_exti(void) {
    // Включение тактирования AFIO (необходимо для EXTI)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    init_washer_exti_config();

    // Настройка контроллера прерываний (NVIC)
    NVIC_InitTypeDef nvic_init = {0};
    nvic_init.NVIC_IRQChannel = EXTI7_0_IRQn; // Канал прерываний 0-7
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0; // Приоритет
    nvic_init.NVIC_IRQChannelSubPriority = 0; // Подприоритет
    nvic_init.NVIC_IRQChannelCmd = ENABLE; // Включить канал
    NVIC_Init(&nvic_init);
}

// Настройка конфигурации прерываний омывателя
void init_washer_exti_config(void) {
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
    exti_init.EXTI_Trigger = EXTI_Trigger_Falling; // Спад сигнала
    exti_init.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init);
}

// ========================================================
// ОБРАБОТЧИК ПРЕРЫВАНИЙ
// ========================================================

void EXTI7_0_IRQHandler(void) {
    // Обработка прерывания от кнопки включения системы (PC0)
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        // Чтение стабильного состояния с защитой от дребезга
        const BitAction state = read_debounced_input(WASHER_ENABLE_PORT, WASHER_ENABLE_PIN, 3, 10);

        // Обновление состояния системы
        configure_washer_state(state == Bit_RESET);

        // Сброс флага прерывания
        EXTI_ClearITPendingBit(EXTI_Line0);
    }

    // Обработка прерывания от кнопки омывателя (PC1)
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        // Чтение стабильного состояния с защитой от дребезга
        const BitAction state = read_debounced_input(WASHER_BUTTON_PORT, WASHER_BUTTON_PIN, 3, 10);

        // Активация клапана при подтвержденном нажатии
        if (state == Bit_RESET) update_valve_state();

        // Сброс флага прерывания
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

// ========================================================
// ФУНКЦИИ УПРАВЛЕНИЯ СИСТЕМОЙ
// ========================================================

// Временная функция защиты от дребезга (заменить на аппаратное решение)
static BitAction read_debounced_input(GPIO_TypeDef *port, uint16_t pin,
                                      uint8_t samples, uint16_t interval) {
    BitAction last_state = GPIO_ReadInputDataBit(port, pin);
    uint8_t stable_count = 0;

    // Проверка стабильности состояния в течение нескольких циклов
    while (stable_count < samples) {
        Delay_Ms(interval);
        const BitAction current_state = GPIO_ReadInputDataBit(port, pin);

        if (current_state == last_state) stable_count++;
        else {
            last_state = current_state;
            stable_count = 0;
        }
    }
    return last_state;
}

// Управление состоянием омывателя (включение/выключение)
void configure_washer_state(const uint8_t enabled) {
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
void update_valve_state(void) {
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
