#include "ch32v00x_gpio.h"
#include "ch32v00x_rcc.h"
#include "ch32v00x_exti.h"
#include "ch32v00x_misc.h"

#define LED_PIN     GPIO_Pin_1
#define LED_PORT    GPIOC
#define BUTTON_PIN  GPIO_Pin_0
#define BUTTON_PORT GPIOC

// Флаг для отслеживания состояния светодиода
volatile uint8_t led_state = 0;

// Прототип обработчика прерывания
void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

// Конфигурация GPIO
void GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // Включаем тактирование порта C
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // Настройка кнопки (PC0) как вход с подтяжкой к питанию
    GPIO_InitStructure.GPIO_Pin = BUTTON_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // Input Pull-Up
    GPIO_Init(BUTTON_PORT, &GPIO_InitStructure);

    // Настройка светодиода (PC1) как выход Push-Pull
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);

    // Изначально выключаем светодиод
    GPIO_SetBits(LED_PORT, LED_PIN);
}

// Конфигурация EXTI и NVIC
void EXTI_Config(void) {
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    // Включаем тактирование AFIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // Настраиваем EXTI на линии 0 (PC0)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);

    // Конфигурация прерывания по срезу (нажатие кнопки)
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Настраиваем NVIC для обработки прерываний EXTI0-7
    NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// Обработчик прерывания для EXTI0-7
void EXTI7_0_IRQHandler(void) {
    // Проверяем, что прерывание вызвано кнопкой (PC0)
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        // Инвертируем состояние светодиода
        led_state = !led_state;

        // Устанавливаем новое состояние светодиода
        if (led_state) {
            GPIO_ResetBits(LED_PORT, LED_PIN); // Включить
        } else {
            GPIO_SetBits(LED_PORT, LED_PIN); // Выключить
        }

        // Задержка для подавления дребезга
        for (volatile uint32_t i = 0; i < 20000; i++); // ~20ms

        // Сбрасываем флаг прерывания
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

int main(void) {
    // Инициализация системы
    SystemInit();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    // Настройка периферии
    GPIO_Config();
    EXTI_Config();

    // Бесконечный цикл
    while (1) {
        Delay_Ms(1000);
    }
}
