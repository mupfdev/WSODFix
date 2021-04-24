// Spdx-License-Identifier: MIT
/**
 * @file    main.c
 * @brief   WSODFix WSOD fixer for the Nokia N-Gage
 * @details Fixes the WSOD problem by formatting the user area via FBus
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

typedef enum
{
    FBUS_HANDSHAKE        = 0x15,
    FBUS_ACK              = 0x7F,
    FBUS_FORMAT_USER_AREA = 0x58

} fbus_command;

typedef enum
{
    SEND_SYNC = 0,        /* Send [55 55 55 55 55 55]                                   */
    SEND_HANDSHAKE,       /* Send [1E 00 10 15 00 08] [00 06 00 02 00 00 01 60] [0F 79] */
    RECV_HANDSHAKE_ACK,   /* Recv [1E 10 00 7F 00 02] [15 00]                   [0B 6D] */
    RECV_HANDSHAKE,       /* Recv [1E 10 00 15 00 08] [06 27 00 65 05 05 01 xx] [xx xx] */
    SEND_HANDSHAKE_ACK,   /* Send [1E 00 10 7F 00 02] [15 xx]                   [xx xx] */
    SEND_FORMAT_START,    /* Send [1E 00 10 58 00 08] [00 0B 00 07 06 00 01 41] [09 1D] */
    RECV_FORMAT_ACK,      /* Recv [1E 10 00 7F 00 02] [58 01]                   [46 6C] */
    RECV_SYNC,            /* Recv [55 55]                                               */
    RECV_FORMAT_DONE,     /* Recv [1E 10 00 58 00 08] [0B 38 00 08 00 00 01 xx] [xx xx] */
    SEND_FORMAT_DONE_ACK, /* Send [1E 00 10 7F 00 02] [58 xx]                   [xx xx] */
    HALT_ERROR

} app_progress;

typedef enum
{
    LED_OK = 0,
    LED_ERROR,
    LED_WAIT_FOR_ACK,
    LED_FORMATTING

} led_pattern;

UART_HandleTypeDef huart3;
TIM_HandleTypeDef  htim4;

static uint8_t      rx_buffer[16] = { 0 };
static app_progress progress      = SEND_SYNC;
static led_pattern  led_gn        = LED_OK;

static void calculate_crc(uint8_t buffer[16], uint8_t* even_crc, uint8_t* odd_crc, bool append);
static void error_handler(void);
static bool is_message_valid(uint8_t buffer[16]);
static int  send_command(fbus_command command, uint8_t payload_size, uint8_t* payload);
static int  sync_fbus(void);
static void system_init(void);

int main(void)
{
    system_init();

    while(1)
    {
        switch (progress)
        {
            case SEND_SYNC:
                if (0 > sync_fbus())
                {
                    error_handler();
                }
                HAL_Delay(60);
                progress = SEND_HANDSHAKE;
                continue;
            case SEND_HANDSHAKE:
            {
                uint8_t payload[8] = { 0x00, 0x06, 0x00, 0x02, 0x00, 0x00, 0x01, 0x60 };
                if (0 > send_command(FBUS_HANDSHAKE, 8, payload))
                {
                    error_handler();
                }
                progress = RECV_HANDSHAKE_ACK;
                continue;
            }
            case RECV_HANDSHAKE_ACK:
                led_gn = LED_WAIT_FOR_ACK;
                HAL_UART_Receive_IT(&huart3, rx_buffer, 10);
                break;
            case RECV_HANDSHAKE:
                /* tbd. */
                break;
            case SEND_HANDSHAKE_ACK:
                /* tbd. */
                break;
            case SEND_FORMAT_START:
                /* tbd. */
                break;
            case RECV_FORMAT_ACK:
                /* tbd. */
                break;
            case RECV_SYNC:
                /* tbd. */
                break;
            case RECV_FORMAT_DONE:
                /* tbd. */
                break;
            case SEND_FORMAT_DONE_ACK:
                /* tbd. */
                break;
            case HALT_ERROR:
                led_gn = LED_ERROR;
                break;
        }

        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        switch (led_gn)
        {
            case LED_ERROR:
                HAL_Delay(100);
                break;
            case LED_FORMATTING:
                HAL_Delay(60);
                break;
            case LED_OK:
            case LED_WAIT_FOR_ACK:
            default:
                HAL_Delay(500);
                break;
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (RECV_HANDSHAKE_ACK == progress)
    {
        if (true == is_message_valid(rx_buffer))
        {
            if (rx_buffer[0] == 0x1E &&
                rx_buffer[3] == FBUS_ACK &&
                rx_buffer[6] == FBUS_HANDSHAKE)
            {
                progress = RECV_HANDSHAKE;
            }
            else
            {
                progress = HALT_ERROR;
            }
        }
        else
        {
            progress = HALT_ERROR;
        }
    }
}

static void calculate_crc(uint8_t buffer[16], uint8_t* even_crc, uint8_t* odd_crc, bool append)
{
    uint8_t payload_size = buffer[5];

    *even_crc = 0;
    *odd_crc  = 0;

    for (int i = 0; i < 6 + payload_size; i += 1)
    {
        if (i % 2 == 0)
        {
            *even_crc ^= buffer[i];
        }
        else
        {
            *odd_crc ^= buffer[i];
        }
    }

    if (true == append)
    {
        buffer[6 + payload_size] = *even_crc;
        buffer[7 + payload_size] = *odd_crc;
    }
}

static void error_handler(void)
{
    __disable_irq();
    while (1) {}
}

static bool is_message_valid(uint8_t buffer[16])
{
    uint8_t payload_size = buffer[5];
    uint8_t even_crc     = 0;
    uint8_t odd_crc      = 0;

    if (buffer[0] != 0x1E)
    {
        return false;
    }

    if (payload_size % 2 != 0 || payload_size > 8)
    {
        return false;
    }

    calculate_crc(buffer, &even_crc, &odd_crc, false);
    if (buffer[6 + payload_size] != even_crc)
    {
        return false;
    }
    if (buffer[7 + payload_size] != odd_crc)
    {
        return false;
    }

    return true;
}

static int send_command(fbus_command command, uint8_t payload_size, uint8_t* payload)
{
    uint8_t even_crc      = 0;
    uint8_t odd_crc       = 0;
    uint8_t tx_buffer[16] = { 0x1E, 0x00, 0x10, command, 0x00 };

    tx_buffer[5] = payload_size;
    memcpy(&tx_buffer[6], &payload[0], payload_size);
    calculate_crc(tx_buffer, &even_crc, &odd_crc, true);

    if (false == is_message_valid(tx_buffer))
    {
        return -2;
    }

    if (HAL_OK != HAL_UART_Transmit(&huart3, tx_buffer, 8 + payload_size, 100))
    {
        return -1;
    }

    return 0;
}

static int sync_fbus(void)
{
    uint8_t tx_buffer[6] = { 0x55, 0x55, 0x55, 0x55, 0x55, 0x55 };

    if (HAL_OK != HAL_UART_Transmit(&huart3, tx_buffer, 6, 100))
    {
        return -1;
    }
    return 0;
}

static void system_init(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    GPIO_InitTypeDef   GPIO_InitStruct   = { 0 };

    /* Reset of all peripherals, Initialises the Flash interface and the
     * Systick.
     */
    HAL_Init();

    /* Initialises the RCC Oscillators according to the specified
     * parameters in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;

    if (HAL_OK != HAL_RCC_OscConfig(&RCC_OscInitStruct))
    {
        error_handler();
    }

    /* Initialises the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_OK != HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2))
    {
        error_handler();
    }

    /* Enable GPIO port clocks */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    /* Configure GPIO pin : PC13 */
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Initialise UART */
    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 115200;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_OK != HAL_UART_Init(&huart3))
    {
        error_handler();
    }
}

void HAL_MspInit(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /* System interrupt init */
    /* DISABLE: JTAG-DP Disabled and SW-DP Disabled */
    __HAL_AFIO_REMAP_SWJ_DISABLE();
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    if(USART3 == huart->Instance)
    {
        /* Peripheral clock enable */
        __HAL_RCC_USART3_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /* USART3 GPIO Configuration
         *   PB10 --> USART3_TX
         *   PB11 --> USART3_RX
         */
        GPIO_InitStruct.Pin   = GPIO_PIN_10;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USART3 interrupt Init */
        HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
    if(huart->Instance==USART3)
    {
        /* Peripheral clock disable */
        __HAL_RCC_USART3_CLK_DISABLE();

        /* USART3 GPIO Configuration
         *   PB10 --> USART3_TX
         *   PB11 --> USART3_RX
         */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);

        /* USART3 interrupt DeInit */
        HAL_NVIC_DisableIRQ(USART3_IRQn);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (TIM4 == htim->Instance)
    {
        HAL_IncTick();
    }
}

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    RCC_ClkInitTypeDef clkconfig;
    uint32_t           uwTimclock       = 0;
    uint32_t           uwPrescalerValue = 0;
    uint32_t           pFLatency;

    /* Configure the TIM4 IRQ priority */
    HAL_NVIC_SetPriority(TIM4_IRQn, TickPriority ,0);

    /* Enable the TIM4 global Interrupt */
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    /* Enable TIM4 clock */
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* Get clock configuration */
    HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

    /* Compute TIM4 clock */
    uwTimclock = 2*HAL_RCC_GetPCLK1Freq();
    /* Compute the prescaler value to have TIM4 counter clock equal to
     * 1MHz
     */
    uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

    /* Initialize TIM4 */
    htim4.Instance = TIM4;

    /* Initialize TIMx peripheral as follow:
     * + Period = [(TIM4CLK/1000) - 1]. to have a (1/1000) s time base.
     * + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
     * + ClockDivision = 0
     * + Counter direction = Up
     */
    htim4.Init.Period = (1000000U / 1000U) - 1U;
    htim4.Init.Prescaler = uwPrescalerValue;
    htim4.Init.ClockDivision = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    if(HAL_TIM_Base_Init(&htim4) == HAL_OK)
    {
        /* Start the TIM time Base generation in interrupt mode */
        return HAL_TIM_Base_Start_IT(&htim4);
    }

    /* Return function status */
    return HAL_ERROR;
}

void HAL_SuspendTick(void)
{
    /* Disable TIM4 update Interrupt */
    __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
}

void HAL_ResumeTick(void)
{
    /* Enable TIM4 Update interrupt */
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
}

/* Cortex-M3 Processor Interruption and Exception Handlers */
void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}

/*
 * STM32F1xx Peripheral Interrupt Handlers
 * Add here the Interrupt Handlers for the used peripherals.
 * For the available peripheral interrupt handler names,
 * please refer to the startup file (startup_stm32f1xx.s).
 */
void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}
