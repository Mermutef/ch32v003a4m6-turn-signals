#include "ch32v00x_gpio.h"

/* Global define */
#define LED_PIN GPIO_Pin_1
#define LED_PORT GPIOC

void GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // Enable GPIOC clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // Configure LED pin (PC1) as output push-pull
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);

    // Initially turn off LED
    GPIO_ResetBits(LED_PORT, LED_PIN);
}

int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    // Configure GPIO
    GPIO_Config();

    while (1) {
        // Turn LED on
        GPIO_SetBits(LED_PORT, LED_PIN);
        Delay_Ms(500);

        // Turn LED off
        GPIO_ResetBits(LED_PORT, LED_PIN);
        Delay_Ms(500);
    }
}
