/* Spdx-License-Identifier: MIT */
/**
 * @file main.c
 * @brief WSODFix A repair tool for Symbian Nokia phones affected by the
 *        infamous white screen of death.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

#define BUF_SIZE     32
#define FBUS_MSG_LEN 16
#define FBUS_RX_ADDR 0x10
#define FBUS_TX_ADDR 0x00

typedef enum
{
    LED_IDLE               = 500,
    LED_ENTER_SERVICE_MODE = 1000,
    LED_FORMATTING         = 30,
    LED_ALL_DONE           = 0

} led_t;

UART_HandleTypeDef huart3;
TIM_HandleTypeDef  htim2;
TIM_HandleTypeDef  htim4;

static bool    all_done        = false;
static bool    initiate_format = false;
static uint8_t rx_byte         = 0;
static led_t   status_led      = LED_IDLE;
static uint8_t frame_counter   = 0;
static uint8_t tx_buffer[16]   = {
    0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
    0x00, 0x06, 0x00, 0x02, 0x00, 0x00, 0x01, 0x60,
    0x0F, 0x79
};

static void system_init(void);
static void service_mode_init(void);
static void comm_init(void);
static void calculate_crc(uint8_t buffer[16], uint8_t* even_crc, uint8_t* odd_crc, bool append);
static void error_handler(void);

int main(void)
{
    system_init();
    service_mode_init();
    comm_init();

    while (1) {}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t fbus_message[FBUS_MSG_LEN] = { 0 };
    uint8_t fbus_header_index          = 0;
    bool    fbus_header_found          = false;

    static uint8_t ring_buffer[BUF_SIZE] = { 0 };
    static uint8_t ring_buffer_index     = 0;

    ring_buffer[ring_buffer_index] = rx_byte;

    for (uint8_t index = 0; index < BUF_SIZE; index += 1)
    {
        if (0x1E == ring_buffer[index])
        {
            fbus_header_index = index;
            fbus_header_found = true;
            break;
        }
    }

    if (true == fbus_header_found)
    {
        uint8_t payload_size  = 0;
        uint8_t even_crc      = 0;
        uint8_t odd_crc       = 0;
        uint8_t message_index = fbus_header_index;

        /* Extract FBus message */
        for (uint8_t index = 0; index < FBUS_MSG_LEN; index += 1)
        {
            fbus_message[index]  = ring_buffer[message_index];
            message_index       += 1;

            if (BUF_SIZE <= message_index)
            {
                message_index = 0;
            }
        }
        payload_size = fbus_message[5];

        /* Validate FBus message */
        if (0 == payload_size || payload_size > 8)
        {
            /* Invalid payload size */
        }
        else
        {
            calculate_crc(fbus_message, &even_crc, &odd_crc, false);

            if ((fbus_message[6 + payload_size] == even_crc) &&
                (fbus_message[7 + payload_size] == odd_crc))
            {
                /* Valid message found */
                /* Check if message is for us */
                if (FBUS_RX_ADDR == fbus_message[1])
                {
                    bool send_ack = true;
                    switch (fbus_message[3])
                    {
                        case 0x7F:
                            send_ack = false;
                            break;
                        case 0x58:
                            all_done = true;
                            break;
                        default:
                            frame_counter = fbus_message[5 + payload_size];
                            break;
                    }

                    if (true == send_ack)
                    {
                        uint8_t ack_buffer[10] = {
                            0x1E, FBUS_TX_ADDR, FBUS_RX_ADDR, 0x7F,
                            0x00, 0x02, fbus_message[3], frame_counter - 0x40
                        };
                        calculate_crc(ack_buffer, &even_crc, &odd_crc, true);
                        HAL_UART_Transmit_IT(&huart3, ack_buffer, 10);

                        if (0x15 == fbus_message[3])
                        {
                            initiate_format = true;
                        }
                    }
                }
                else
                {
                    /* Nothing to do here; message can be ignored */
                }

                /* Invalidate processed message */
                ring_buffer[fbus_header_index] = 0x00;
            }
        }
    }

    ring_buffer_index += 1;
    if (BUF_SIZE <= ring_buffer_index)
    {
        ring_buffer_index = 0;
    }
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Handshake acknowledge sent: initiate user area format */
    if (true == initiate_format)
    {
        tx_buffer[3]  = 0x58;
        tx_buffer[4]  = 0x00;
        tx_buffer[5]  = 0x08;
        tx_buffer[6]  = 0x00;
        tx_buffer[7]  = 0x0B;
        tx_buffer[8]  = 0x00;
        tx_buffer[9]  = 0x07;
        tx_buffer[10] = 0x06;
        tx_buffer[11] = 0x00;
        tx_buffer[12] = 0x01;
        tx_buffer[13] = 0x41;
        tx_buffer[14] = 0x09;
        tx_buffer[15] = 0x1D;

        status_led      = LED_FORMATTING;
        initiate_format = false;
        HAL_UART_Transmit_IT(&huart3, tx_buffer, 16);
    }
}

static void service_mode_init(void)
{
    led_t prev_pattern = status_led;

    status_led = LED_ENTER_SERVICE_MODE;

    HAL_Delay(10000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(987);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(849);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(592);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(5000);

    status_led = prev_pattern;
}

static void comm_init(void)
{
    if (HAL_OK != HAL_UART_Transmit(&huart3, tx_buffer, 6, 100))
    {
        error_handler();
    }
    HAL_Delay(60);

    tx_buffer[0] = 0x1E;
    tx_buffer[1] = FBUS_TX_ADDR;
    tx_buffer[2] = FBUS_RX_ADDR;
    tx_buffer[3] = 0x15;
    tx_buffer[4] = 0x00;
    tx_buffer[5] = 0x08;
    if (HAL_OK != HAL_UART_Transmit(&huart3, tx_buffer, 16, 100))
    {
        error_handler();
    }

    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
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
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,  GPIO_PIN_RESET);

    /* Configure GPIO pin : PC13 */
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Configure GPIO pin : PB12 */
    GPIO_InitStruct.Pin   = GPIO_PIN_12;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configure GPIO pin : PB1 */
    GPIO_InitStruct.Pin   = GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 72-1;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 1000-1;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.RepetitionCounter = 0;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_OK != HAL_TIM_Base_Init(&htim2))
    {
        error_handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_OK != HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig))
    {
        error_handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_OK != HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig))
    {
        error_handler();
    }

    /* Start the TIM time Base generation in interrupt mode */
    if (HAL_OK != HAL_TIM_Base_Start_IT(&htim2))
    {
        error_handler();
    }
}

static void error_handler(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    __disable_irq();
    while (1) {}
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

    if (USART3 == huart->Instance)
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
        HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
    if (USART3 == huart->Instance)
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
    if (TIM2 == htim_base->Instance)
    {
        /* Peripheral clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();

        /* TIM2 interrupt Init */
        HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
    if (TIM2 == htim_base->Instance)
    {
        /* Peripheral clock disable */
        __HAL_RCC_TIM2_CLK_DISABLE();

        /* TIM2 interrupt DeInit */
        HAL_NVIC_DisableIRQ(TIM2_IRQn);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (TIM2 == htim->Instance)
    {
        if (true == all_done)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        }
        else
        {
            static uint32_t ms_count = 0;
            ms_count += 1;
            if (ms_count >= (uint32_t)status_led)
            {
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
                ms_count = 0;
            }
        }
    }

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
    HAL_NVIC_SetPriority(TIM4_IRQn, TickPriority, 0);

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

    if (HAL_OK == HAL_TIM_Base_Init(&htim4))
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
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}
