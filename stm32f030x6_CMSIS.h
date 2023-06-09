/**
 ******************************************************************************
 *  @file stm32f030x6_CMSIS.h
 *  @brief CMSIS на примере МК STM32F030K6T6
 *  @author Волков Олег
 *  @date 10.04.2023
 *
  ******************************************************************************
 * @attention
 *
 *  Библиотека помогает разобраться с библиотекой CMSIS на примере
 *  МК STM32F030K6T6
 *
 *  YouTube: https://www.youtube.com/channel/UCzZKTNVpcMSALU57G1THoVw
 *  GitHub: https://github.com/Solderingironspb
 *  Группа ВК: https://vk.com/solderingiron.stm32
 *  Reference Manual: https://www.st.com/resource/en/reference_manual/dm00091010-stm32f030x4x6x8xc-and-stm32f070x6xb-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
 *  Datasheet: https://www.st.com/resource/en/datasheet/stm32f030f4.pdf
 *  Programming manual: https://www.st.com/resource/en/programming_manual/pm0215-stm32f0-series-cortexm0-programming-manual-stmicroelectronics.pdf
 ******************************************************************************
 */

#include "main.h"
#include <stdbool.h>

void CMSIS_Debug_init(void);
void CMSIS_RCC_SystemClock_48MHz(void);
void CMSIS_SysTick_Timer_init(void);
void Delay_ms(uint32_t Milliseconds);
void SysTick_Handler(void);
void CMSIS_PB0_OUTPUT_Push_Pull_init(void);
void CMSIS_PB1_OUTPUT_Push_Pull_init(void);
void CMSIS_PB7_OUTPUT_Push_Pull_init(void);
void CMSIS_TIM3_init(void);
void TIM3_IRQHandler(void);
void CMSIS_TIM14_init(void);
void CMSIS_ADC_DMA_init(void);
void CMSIS_USART1_Init(void);
void USART1_IRQHandler(void);
bool CMSIS_USART_Transmit(USART_TypeDef* USART, uint8_t* data, uint16_t Size, uint32_t Timeout_ms);
void FLASH_Unlock(void);
void FLASH_Lock(void);
void FLASH_Page_erase(uint16_t Adress);
void FLASH_Page_write(uint32_t Adress, uint8_t *Data, uint16_t Size);
void FLASH_Read_data(uint32_t Adress, uint8_t *Data, uint16_t Size);
