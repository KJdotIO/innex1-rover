#include <stdint.h>

// ============================================================
// STM32F401RE Register Addresses
// ============================================================

#define RCC_BASE        0x40023800UL
#define GPIOA_BASE      0x40020000UL
#define TIM2_BASE       0x40000000UL
#define USART2_BASE     0x40004400UL
#define SYSTICK_BASE    0xE000E010UL

// RCC registers
#define RCC_AHB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x40))

// GPIOA registers
#define GPIOA_MODER     (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_OTYPER    (*(volatile uint32_t *)(GPIOA_BASE + 0x04))
#define GPIOA_PUPDR     (*(volatile uint32_t *)(GPIOA_BASE + 0x0C))
#define GPIOA_ODR       (*(volatile uint32_t *)(GPIOA_BASE + 0x14))
#define GPIOA_AFRL      (*(volatile uint32_t *)(GPIOA_BASE + 0x20))

// TIM2 registers
#define TIM2_CR1        (*(volatile uint32_t *)(TIM2_BASE + 0x00))
#define TIM2_CCMR1      (*(volatile uint32_t *)(TIM2_BASE + 0x18))
#define TIM2_CCER       (*(volatile uint32_t *)(TIM2_BASE + 0x20))
#define TIM2_PSC        (*(volatile uint32_t *)(TIM2_BASE + 0x28))
#define TIM2_ARR        (*(volatile uint32_t *)(TIM2_BASE + 0x2C))
#define TIM2_CCR1       (*(volatile uint32_t *)(TIM2_BASE + 0x34))
#define TIM2_EGR        (*(volatile uint32_t *)(TIM2_BASE + 0x14))

// USART2 registers
#define USART2_SR       (*(volatile uint32_t *)(USART2_BASE + 0x00))
#define USART2_DR       (*(volatile uint32_t *)(USART2_BASE + 0x04))
#define USART2_BRR      (*(volatile uint32_t *)(USART2_BASE + 0x08))
#define USART2_CR1      (*(volatile uint32_t *)(USART2_BASE + 0x0C))

// SysTick registers
#define SYST_CSR        (*(volatile uint32_t *)(SYSTICK_BASE + 0x00))
#define SYST_RVR        (*(volatile uint32_t *)(SYSTICK_BASE + 0x04))
#define SYST_CVR        (*(volatile uint32_t *)(SYSTICK_BASE + 0x08))

// ============================================================
// Pin Definitions
// ============================================================

#define PWM_PIN         5   // PA5 / D13 -> Cytron MDD10A PWM1
#define DIR_PIN         6   // PA6 / D12 -> Cytron MDD10A DIR1

// ============================================================
// Door Actuator Settings
// Tune these values on the rover
// ============================================================

#define ACTUATOR_SPEED_PERCENT  50

#define DOOR_OPEN_TIME_MS       8000
#define DOOR_CLOSE_TIME_MS      8000

#define DIRECTION_OPEN          1
#define DIRECTION_CLOSE         0

// ============================================================
// Door State Machine
// ============================================================

typedef enum
{
    DOOR_UNKNOWN = 0,
    DOOR_OPENING,
    DOOR_CLOSING,
    DOOR_OPEN,
    DOOR_CLOSED,
    DOOR_STOPPED
} DoorState;

volatile uint32_t system_ms = 0;

DoorState door_state = DOOR_UNKNOWN;
uint32_t door_action_start_ms = 0;

// ============================================================
// Millisecond Timer Using SysTick
// ============================================================

void SysTick_Handler(void)
{
    system_ms++;
}

uint32_t millis(void)
{
    return system_ms;
}

void SysTick_Init(void)
{
    /*
        Assumes default 16 MHz system clock.
        16,000 clock ticks = 1 millisecond.
    */
    SYST_RVR = 16000 - 1;
    SYST_CVR = 0;

    /*
        Bit 0 = ENABLE
        Bit 1 = TICKINT, enable interrupt
        Bit 2 = CLKSOURCE, use processor clock
    */
    SYST_CSR = 7;
}

// ============================================================
// PWM Setup: PA5 / D13 / TIM2_CH1
// ============================================================

void PWM_Init(void)
{
    // Enable GPIOA clock
    RCC_AHB1ENR |= (1U << 0);

    // Enable TIM2 clock
    RCC_APB1ENR |= (1U << 0);

    // Set PA5 to alternate function mode
    GPIOA_MODER &= ~(3U << (PWM_PIN * 2));
    GPIOA_MODER |=  (2U << (PWM_PIN * 2));

    // Set PA5 to AF1, which is TIM2_CH1
    GPIOA_AFRL &= ~(0xFU << (PWM_PIN * 4));
    GPIOA_AFRL |=  (1U << (PWM_PIN * 4));

    /*
        PWM frequency = 16 MHz / (PSC + 1) / (ARR + 1)
        16 MHz / 16 / 1000 = 1000 Hz
    */
    TIM2_PSC = 15;
    TIM2_ARR = 999;

    // PWM mode 1 on channel 1
    TIM2_CCMR1 &= ~(7U << 4);
    TIM2_CCMR1 |=  (6U << 4);

    // Enable preload for channel 1
    TIM2_CCMR1 |= (1U << 3);

    // Enable channel 1 output
    TIM2_CCER |= (1U << 0);

    // Start stopped
    TIM2_CCR1 = 0;

    // Update timer registers
    TIM2_EGR |= (1U << 0);

    // Enable timer
    TIM2_CR1 |= (1U << 0);
}

void Set_PWM_Percent(uint8_t percent)
{
    if (percent > 100)
    {
        percent = 100;
    }

    TIM2_CCR1 = (percent * 999) / 100;
}

// ============================================================
// Direction Pin Setup: PA6 / D12
// ============================================================

void DIR_Init(void)
{
    // Enable GPIOA clock
    RCC_AHB1ENR |= (1U << 0);

    // Set PA6 as output
    GPIOA_MODER &= ~(3U << (DIR_PIN * 2));
    GPIOA_MODER |=  (1U << (DIR_PIN * 2));

    // Push-pull output
    GPIOA_OTYPER &= ~(1U << DIR_PIN);

    // No pull-up / pull-down
    GPIOA_PUPDR &= ~(3U << (DIR_PIN * 2));
}

void Set_Direction(uint8_t direction)
{
    if (direction)
    {
        GPIOA_ODR |= (1U << DIR_PIN);
    }
    else
    {
        GPIOA_ODR &= ~(1U << DIR_PIN);
    }
}

// ============================================================
// USART2 Setup
// Used for Jetson/laptop -> Nucleo commands
// PA2 = TX, PA3 = RX
// ============================================================

void USART2_Init(void)
{
    // Enable GPIOA clock
    RCC_AHB1ENR |= (1U << 0);

    // Enable USART2 clock
    RCC_APB1ENR |= (1U << 17);

    // PA2 = USART2_TX
    GPIOA_MODER &= ~(3U << (2 * 2));
    GPIOA_MODER |=  (2U << (2 * 2));

    // PA3 = USART2_RX
    GPIOA_MODER &= ~(3U << (3 * 2));
    GPIOA_MODER |=  (2U << (3 * 2));

    // AF7 for USART2 on PA2
    GPIOA_AFRL &= ~(0xFU << (2 * 4));
    GPIOA_AFRL |=  (7U << (2 * 4));

    // AF7 for USART2 on PA3
    GPIOA_AFRL &= ~(0xFU << (3 * 4));
    GPIOA_AFRL |=  (7U << (3 * 4));

    // Baud rate 9600 assuming 16 MHz clock
    USART2_BRR = 0x0683;

    // Enable RX, TX, USART
    USART2_CR1 = (1U << 2) | (1U << 3) | (1U << 13);
}

uint8_t USART2_ByteAvailable(void)
{
    // RXNE bit = bit 5
    return (USART2_SR & (1U << 5)) != 0;
}

char USART2_ReadByte(void)
{
    return (char)(USART2_DR & 0xFF);
}

void USART2_SendByte(char data)
{
    // TXE bit = bit 7
    while (!(USART2_SR & (1U << 7)))
    {
    }

    USART2_DR = data;
}

void USART2_SendString(const char *str)
{
    while (*str)
    {
        USART2_SendByte(*str);
        str++;
    }
}

// ============================================================
// Door Actuator Module
// ============================================================

void Door_Stop(void)
{
    Set_PWM_Percent(0);
    door_state = DOOR_STOPPED;

    USART2_SendString("Door stopped\r\n");
}

void Door_Open(void)
{
    Set_Direction(DIRECTION_OPEN);
    Set_PWM_Percent(ACTUATOR_SPEED_PERCENT);

    door_action_start_ms = millis();
    door_state = DOOR_OPENING;

    USART2_SendString("Door opening\r\n");
}

void Door_Close(void)
{
    Set_Direction(DIRECTION_CLOSE);
    Set_PWM_Percent(ACTUATOR_SPEED_PERCENT);

    door_action_start_ms = millis();
    door_state = DOOR_CLOSING;

    USART2_SendString("Door closing\r\n");
}

void Door_Toggle(void)
{
    if (door_state == DOOR_OPEN || door_state == DOOR_OPENING)
    {
        Door_Close();
    }
    else
    {
        Door_Open();
    }
}

void Door_Update(void)
{
    uint32_t now = millis();

    if (door_state == DOOR_OPENING)
    {
        if ((now - door_action_start_ms) >= DOOR_OPEN_TIME_MS)
        {
            Set_PWM_Percent(0);
            door_state = DOOR_OPEN;

            USART2_SendString("Door open\r\n");
        }
    }
    else if (door_state == DOOR_CLOSING)
    {
        if ((now - door_action_start_ms) >= DOOR_CLOSE_TIME_MS)
        {
            Set_PWM_Percent(0);
            door_state = DOOR_CLOSED;

            USART2_SendString("Door closed\r\n");
        }
    }
}

// ============================================================
// Command Handler
// ============================================================

void Handle_Command(char command)
{
    if (command == 'o' || command == 'O')
    {
        Door_Open();
    }
    else if (command == 'c' || command == 'C')
    {
        Door_Close();
    }
    else if (command == 's' || command == 'S')
    {
        Door_Stop();
    }
    else if (
        command == 't'
        || command == 'T'
        || command == 'b'
        || command == 'B'
    )
    {
        Door_Toggle();
    }
    else if (command == '\r' || command == '\n')
    {
        // Ignore enter/newline characters
    }
    else
    {
        USART2_SendString("Unknown command\r\n");
    }
}

// ============================================================
// Main Program
// ============================================================

int main(void)
{
    SysTick_Init();
    DIR_Init();
    PWM_Init();
    USART2_Init();

    // Always start safely stopped
    Set_PWM_Percent(0);
    door_state = DOOR_STOPPED;

    USART2_SendString("\r\nDoor actuator controller ready\r\n");
    USART2_SendString(
        "Commands: o=open, c=close, s=stop, t/b=toggle\r\n"
    );

    while (1)
    {
        if (USART2_ByteAvailable())
        {
            char command = USART2_ReadByte();
            Handle_Command(command);
        }

        Door_Update();
    }
}
