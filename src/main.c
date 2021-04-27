// Spdx-License-Identifier: MIT
/**
 * @file    main.c
 * @brief   WSODFix WSOD fixer for Symbian phones by Nokia
 * @details Fixes the WSOD problem by formatting the user area via FBus
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

typedef enum
{
    FBUS_HANDSHAKE = 0x15,
    FBUS_ACK       = 0x7F,
    FBUS_FORMAT    = 0x58

} fbus_command;

typedef enum
{
    SEND_SYNC = 0,           /* Send [55 55 55 55 55 55]                                   */
    SEND_HANDSHAKE,          /* Send [1E 00 10 15 00 08] [00 06 00 02 00 00 01 60] [0F 79] */
    RECV_ACK_RECV_HANDSHAKE, /* Recv [1E 10 00 7F 00 02] [15 00]                   [0B 6D] */
                             /* Recv [1E 10 00 15 00 08] [06 27 00 65 05 05 01 xx] [1C xx] */
    SEND_HANDSHAKE_ACK,      /* Send [1E 00 10 7F 00 02] [15 xx]                   [1B xx] */
    SEND_FORMAT,             /* Send [1E 00 10 58 00 08] [00 0B 00 07 06 00 01 41] [09 1D] */
    RECV_FORMAT_ACK,         /* Recv [1E 10 00 7F 00 02] [58 01]                   [46 6C] */
    RECV_SYNC_RECV_FORMAT,   /* Recv [55 55]                                               */
                             /* Recv [1E 10 00 58 00 08] [0B 38 00 08 00 00 01 xx] [14 xx] */
    SEND_FORMAT_ACK,         /* Send [1E 00 10 7F 00 02] [58 xx]                   [56 xx] */
    ALL_DONE,
    HALT_ERROR

} app_progress;

typedef enum
{
    LED_OK = 0,
    LED_ERROR,
    LED_FORMATTING,

} led_pattern;

UART_HandleTypeDef huart3;
TIM_HandleTypeDef  htim1;
TIM_HandleTypeDef  htim4;

static uint8_t      rx_buffer[26] = { 0 };
static int          frame_counter = 0;
static app_progress progress      = SEND_SYNC;
static led_pattern  status_led    = LED_OK;

static void calculate_crc(uint8_t* buffer, uint8_t* even_crc, uint8_t* odd_crc, bool append);
static void error_handler(void);
static bool is_message_valid(uint8_t* buffer);
static int  send_command(fbus_command command, uint8_t payload_size, uint8_t* payload);
static int  sync_fbus(void);
static void system_init(void);

int main(void)
{
    uint32_t toggle_cnt = 0;
    uint32_t toggle_ms  = 500;

    system_init();

    /* Enter service mode. */
    HAL_Delay(2);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(7700);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(987);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(849);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(592);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(4000);

    while(1)
    {
        switch (progress)
        {
            case HALT_ERROR:
                status_led = LED_ERROR;
                break;
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
                progress = RECV_ACK_RECV_HANDSHAKE;
                continue;
            }
            case RECV_ACK_RECV_HANDSHAKE:
                HAL_UART_Receive_IT(&huart3, &rx_buffer[0], 26);
                break;
            case SEND_HANDSHAKE_ACK:
            {
                uint8_t payload[2] = { FBUS_HANDSHAKE, frame_counter - 0x40 };
                if (0 > send_command(FBUS_ACK, 2, payload))
                {
                    error_handler();
                }
                progress = SEND_FORMAT;
                continue;
            }
            case SEND_FORMAT:
            {
                uint8_t payload[8] = { 0x00, 0x0B, 0x00, 0x07, 0x06, 0x00, 0x01, 0x41 };
                HAL_Delay(16);
                if (0 > send_command(FBUS_FORMAT, 8, payload))
                {
                    error_handler();
                }
                progress = RECV_FORMAT_ACK;
                continue;
            }
            case RECV_FORMAT_ACK:
                HAL_UART_Receive_IT(&huart3, &rx_buffer[0], 10);
                break;
            case RECV_SYNC_RECV_FORMAT:
                HAL_UART_Receive_IT(&huart3, &rx_buffer[0], 18);
                break;
            case SEND_FORMAT_ACK:
            {
                uint8_t payload[2] = { FBUS_FORMAT, frame_counter - 0x40 };
                if (0 > send_command(FBUS_ACK, 2, payload))
                {
                    error_handler();
                }
                progress = ALL_DONE;
                continue;
            }
            case ALL_DONE:
                goto all_done;
            default:
                break;
        }

        switch (status_led)
        {
            case LED_ERROR:
                toggle_ms = 100;
                break;
            case LED_FORMATTING:
                toggle_ms = 30;
                break;
            case LED_OK:
            default:
                toggle_ms = 500;
                break;
        }

        if (__HAL_TIM_GET_COUNTER(&htim1) >= 1000)
        {
            toggle_cnt += 1;
            __HAL_TIM_SET_COUNTER(&htim1, 0);
        }

        if (toggle_cnt >= toggle_ms)
        {
            toggle_cnt = 0;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }
    }

all_done:
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    while (1) {}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (RECV_ACK_RECV_HANDSHAKE == progress)
    {
        uint8_t* msg_ack       = &rx_buffer[0];
        uint8_t* msg_handshake = &rx_buffer[10];

        if (false == is_message_valid(msg_ack) ||
            false == is_message_valid(msg_handshake))
        {
            progress = HALT_ERROR;
            return;
        }

        if (msg_ack[0]        == 0x1E &&
            msg_ack[3]        == FBUS_ACK &&
            msg_ack[6]        == FBUS_HANDSHAKE &&
            msg_handshake[0]  == 0x1E &&
            msg_handshake[3]  == FBUS_HANDSHAKE &&
            msg_handshake[6]  == 0x06 &&
            msg_handshake[7]  == 0x27 &&
            msg_handshake[8]  == 0x00 &&
            msg_handshake[9]  == 0x65 &&
            msg_handshake[10] == 0x05 &&
            msg_handshake[11] == 0x05 &&
            msg_handshake[12] == 0x01)
        {
            frame_counter = msg_handshake[13];
            progress      = SEND_HANDSHAKE_ACK;
            return;
        }
        else
        {
            progress = HALT_ERROR;
            return;
        }
    }
    else if (RECV_FORMAT_ACK == progress)
    {
        uint8_t* msg_ack = &rx_buffer[0];

        if (false == is_message_valid(msg_ack))
        {
            progress = HALT_ERROR;
            return;
        }

        if (msg_ack[0]        == 0x1E &&
            msg_ack[3]        == FBUS_ACK &&
            msg_ack[6]        == FBUS_FORMAT)
        {
            progress   = RECV_SYNC_RECV_FORMAT;
            status_led = LED_FORMATTING;
        }
        else
        {
            progress = HALT_ERROR;
            return;
        }
    }
    else if (RECV_SYNC_RECV_FORMAT == progress)
    {
        uint8_t* msg_sync   = &rx_buffer[0];
        uint8_t* msg_format = &rx_buffer[2];

        if (false == is_message_valid(msg_format))
        {
            progress = HALT_ERROR;
            return;
        }

        if (msg_sync[0]    == 0x55 &&
            msg_sync[1]    == 0x55 &&
            msg_format[0]  == 0x1E &&
            msg_format[3]  == FBUS_FORMAT &&
            msg_format[6]  == 0x0B &&
            msg_format[7]  == 0x38 &&
            msg_format[8]  == 0x00 &&
            msg_format[9]  == 0x08 &&
            msg_format[10] == 0x00 &&
            msg_format[11] == 0x00 &&
            msg_format[12] == 0x01)
        {
            frame_counter = msg_format[13];
            progress      = SEND_FORMAT_ACK;
            return;
        }
        else
        {
            progress = HALT_ERROR;
            return;
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
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
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
    RCC_OscInitTypeDef      RCC_OscInitStruct  = { 0 };
    RCC_ClkInitTypeDef      RCC_ClkInitStruct  = { 0 };
    GPIO_InitTypeDef        GPIO_InitStruct    = { 0 };
    TIM_ClockConfigTypeDef  sClockSourceConfig = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig      = { 0 };

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
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

    /* Configure GPIO pin : PC13 */
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Configure GPIO pin : PB1 */
    GPIO_InitStruct.Pin   = GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 72-1;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 0xffff-1;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_OK != HAL_TIM_Base_Init(&htim1))
    {
        error_handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_OK != HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig))
    {
        error_handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_OK != HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig))
    {
        error_handler();
    }

    HAL_TIM_Base_Start(&htim1);
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

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
    if(TIM1 == htim_base->Instance)
    {
        /* Peripheral clock enable */
        __HAL_RCC_TIM1_CLK_ENABLE();
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
    if(TIM1 == htim_base->Instance)
    {
        /* Peripheral clock disable */
        __HAL_RCC_TIM1_CLK_DISABLE();
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
    htim4.Init.Period        = (1000000U / 1000U) - 1U;
    htim4.Init.Prescaler     = uwPrescalerValue;
    htim4.Init.ClockDivision = 0;
    htim4.Init.CounterMode   = TIM_COUNTERMODE_UP;
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

/*
 * STM32F1xx Peripheral Interrupt Handlers
 * Add here the Interrupt Handlers for the used peripherals.
 * For the available peripheral interrupt handler names,
 * please refer to the startup file (startup_stm32f1xx.s).
 */
void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}
