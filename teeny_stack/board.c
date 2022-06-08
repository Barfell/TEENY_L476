/*       
 *         _______                    _    _  _____ ____  
 *        |__   __|                  | |  | |/ ____|  _ \ 
 *           | | ___  ___ _ __  _   _| |  | | (___ | |_) |
 *           | |/ _ \/ _ \ '_ \| | | | |  | |\___ \|  _ < 
 *           | |  __/  __/ | | | |_| | |__| |____) | |_) |
 *           |_|\___|\___|_| |_|\__, |\____/|_____/|____/ 
 *                               __/ |                    
 *                              |___/                     
 *
 * TeenyUSB - light weight usb stack for micro controllers
 * 
 * Copyright (c) 2020 XToolBox  - admin@xtoolbox.org
 *                         www.tusb.org
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


////////////////////////////////////////////////
/// TeenyUSB board related API
////////////////////////////////////////////////

#include "stm32l4xx_hal.h"
#include "teeny_usb_def.h"
#include "tusb_dev_drv_stm32_otg.h"
#include <errno.h>
#include <stdio.h>
#include "stm32l4xx_hal_uart.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_system.h"

#define MODE(x, pin)  ((x) << ((pin)*2))
#define TYPE(x, pin)  ((x) << ((pin)))
#define AF(x,  pin)    (((pin)<8) ? ((uint32_t)(x) << ((pin)*4)) : ((uint32_t)(x) << ( ((pin)-8)*4)) )
void SystemCoreClockUpdate(void);
void console_uart_init(uint32_t baud)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
  
    //__HAL_RCC_USART2_CONFIG(RCC_USART2CLKSOURCE_SYSCLK);
  
     /**USART2 GPIO Configuration
        PC11     ------> USART2_RX
        PC10     ------> USART2_TX
     */
    GPIOA->MODER &= ~ (MODE(3,2) | MODE(3,3));
    GPIOA->MODER |=   (MODE(2,2) | MODE(2,3));  
    GPIOA->OTYPER &= ~(TYPE(1,2) | TYPE(1,3));
    GPIOA->OSPEEDR |= (MODE(3,2) | MODE(3,3));
    GPIOA->AFR[1] &= ~(AF(0xf,2) | AF(0xf,3));
    GPIOA->AFR[1] |=  (AF(GPIO_AF7_USART2,2) | AF(GPIO_AF7_USART2,3));
  
    USART2->CR1 &= ~USART_CR1_UE;
  
    MODIFY_REG(USART2->CR2, USART_CR2_STOP, UART_STOPBITS_1);
    
    MODIFY_REG(USART2->CR1, (USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | \
                                   USART_CR1_RE | USART_CR1_OVER8),
                                   UART_WORDLENGTH_8B|UART_PARITY_NONE|UART_MODE_TX_RX|UART_OVERSAMPLING_16);
                                   
    MODIFY_REG(USART2->CR3, (USART_CR3_RTSE | USART_CR3_CTSE), UART_HWCONTROL_NONE);
    
    SystemCoreClockUpdate();
    
    USART2->BRR = UART_DIV_SAMPLING16(SystemCoreClock, baud);//UART_BRR_SAMPLING16(SystemCoreClock/4, baud);
    
    NVIC_SetPriority(USART2_IRQn, 0);
    NVIC_EnableIRQ(USART2_IRQn);
    
    USART2->CR1 |= USART_CR1_RXNEIE;
  
    CLEAR_BIT(USART2->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
    CLEAR_BIT(USART2->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));
    
    USART2->CR1 |=  USART_CR1_UE;
}

/*WEAK uint32_t HAL_GetTick(void)
{
    static __IO uint32_t tick = 0;
    static __IO uint32_t micro_tick = 0;
    
    micro_tick++;
    if(micro_tick > 40){
        tick++;
        micro_tick = 0;
    }
    return tick;
}*/

WEAK void stdin_recvchar(int ch)
{
    (void)ch;
}

void USART2_IRQHandler(void)
{
    uint32_t sr = USART2->ISR;
    if ( (sr & UART_FLAG_RXNE) ){
        char ch = USART2->RDR & 0xff;
        stdin_recvchar(ch);
    }
    if( sr & UART_FLAG_ORE){
        char ch = USART2->RDR & 0xff;
        (void)ch;
    }
}

void stdout_sendchar(int ch)
{
    while ((USART2->ISR & UART_FLAG_TC) == RESET);
    USART2->TDR = (uint32_t)ch;
}

void stdio_init(void)
{
    console_uart_init(115200);
}


//const tusb_stm32_otg_io_cfg_t f407_otg_default_io[] = {
///* OTG_HS_ULPI_D0  */  {GPIOA,  3, GPIO_AF10_OTG_HS},
///* OTG_HS_ULPI_D1  */  {GPIOB,  0, GPIO_AF10_OTG_HS},
///* OTG_HS_ULPI_D2  */  {GPIOB,  1, GPIO_AF10_OTG_HS},
///* OTG_HS_ULPI_D3  */  {GPIOB, 10, GPIO_AF10_OTG_HS},
///* OTG_HS_ULPI_D4  */  {GPIOB, 11, GPIO_AF10_OTG_HS},
///* OTG_HS_ULPI_D5  */  {GPIOB, 12, GPIO_AF10_OTG_HS},
///* OTG_HS_ULPI_D6  */  {GPIOB, 13, GPIO_AF10_OTG_HS},
///* OTG_HS_ULPI_D7  */  {GPIOB,  5, GPIO_AF10_OTG_HS},
///* OTG_HS_ULPI_DIR */  {GPIOI, 11, GPIO_AF10_OTG_HS},
///* OTG_HS_ULPI_STP */  {GPIOC,  0, GPIO_AF10_OTG_HS},
///* OTG_HS_ULPI_NXT */  {GPIOH,  4, GPIO_AF10_OTG_HS},
///* OTG_HS_ULPI_CK  */  {GPIOA,  5, GPIO_AF10_OTG_HS},
//    {0,0,0},
//};


//const tusb_hardware_param_t otg_default_param = {
//    .is_hs_core = 1,
//    .is_high_speed = 1,
//    .is_internal_phy = 0,
//    .dma_enable = 1,
//    .sof_enable = 0,
//    .low_power_enable = 0,
//    .lpm_enable = 0,
//    .battery_charging_enable = 0,
//    
//    .vbus_sensing_enable = 0,
//    .use_dedicated_ep1 = 0,
//    .use_external_vbus = 0,
//    .io_cfgs = f407_otg_default_io,
//};



const tusb_stm32_otg_io_cfg_t l476_otg_fs_io[] = {
/* DM  */  {GPIOA,  11, GPIO_AF10_OTG_FS},
/* DP  */  {GPIOA,  12, GPIO_AF10_OTG_FS},
    {0,0,0},
};


const tusb_hardware_param_t otg_fs_param = {
    .is_hs_core = 0,
    .is_high_speed = 0,
    .is_internal_phy = 1,//0,
    .dma_enable = 0,
    .sof_enable = 0,
    .low_power_enable = 0,
    .lpm_enable = 0,
    .battery_charging_enable = 0,
    
    .vbus_sensing_enable = 0,
    .use_dedicated_ep1 = 0,
    .use_external_vbus = 0,
    .io_cfgs = l476_otg_fs_io,
};


/**
  * @brief System Clock Configuration
  * @retval None
  */
//void SystemClock_Config(void)
//{
//  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
//  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4)
//  {
//  }
//  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
//  LL_RCC_MSI_Enable();

//   /* Wait till MSI is ready */
//  while(LL_RCC_MSI_IsReady() != 1)
//  {

//  }
//  LL_RCC_MSI_EnableRangeSelection();
//  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);
//  LL_RCC_MSI_SetCalibTrimming(0);
//  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2);
//  LL_RCC_PLL_EnableDomain_SYS();
//  LL_RCC_PLL_Enable();

//   /* Wait till PLL is ready */
//  while(LL_RCC_PLL_IsReady() != 1)
//  {

//  }
//  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

//   /* Wait till System clock is ready */
//  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
//  {

//  }
//  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
//  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
//  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
//  LL_SetSystemCoreClock(80000000);

//   /* Update the time base */
////  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
//  {
////    Error_Handler();
//  }
//}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  LL_RCC_PLLSAI1_ConfigDomain_48M(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 24, LL_RCC_PLLSAI1Q_DIV_2);
  LL_RCC_PLLSAI1_EnableDomain_48M();
  LL_RCC_PLLSAI1_Enable();

   /* Wait till PLLSAI1 is ready */
  while(LL_RCC_PLLSAI1_IsReady() != 1)
  {

  }
}

