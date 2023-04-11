/**
 ******************************************************************************
 *  @file stm32f030x6_CMSIS.c
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


#include "stm32f030x6_CMSIS.h"

/*================================= НАСТРОЙКА GPIO ============================================*/

void CMSIS_PB0_OUTPUT_Push_Pull_init(void) {
    /*Настроим PB0 на выход в режиме Push-pull*/
    SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN); //Запустим тактирование порта B
    MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER0_Msk, 0b01 << GPIO_MODER_MODER0_Pos); //01: General purpose output mode
    CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT_0); //0: Output push-pull (reset state)
    MODIFY_REG(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEEDR0_Msk, 0b11 << GPIO_OSPEEDR_OSPEEDR0_Pos); //High speed
}
void CMSIS_PB1_OUTPUT_Push_Pull_init(void) {
    /*Настроим PB1 на выход в режиме Push-pull*/
    SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN); //Запустим тактирование порта B
    MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER1_Msk, 0b01 << GPIO_MODER_MODER1_Pos); //01: General purpose output mode
    CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT_1); //0: Output push-pull (reset state)
    MODIFY_REG(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEEDR1_Msk, 0b11 << GPIO_OSPEEDR_OSPEEDR1_Pos); //High speed
}

void CMSIS_PB7_OUTPUT_Push_Pull_init(void) {
    /*Настроим PB7 на выход в режиме Push-pull*/
    SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN); //Запустим тактирование порта B
    MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER7_Msk, 0b01 << GPIO_MODER_MODER7_Pos); //01: General purpose output mode
    CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT_7); //0: Output push-pull (reset state)
    MODIFY_REG(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEEDR7_Msk, 0b11 << GPIO_OSPEEDR_OSPEEDR7_Pos); //High speed
}


/*================================= НАСТРОЙКА DEBUG ============================================*/

/**
***************************************************************************************
*  @breif Debug port mapping
***************************************************************************************
*/
void CMSIS_Debug_init(void) {
    /*Заблокируем доступ для редактирования конфигурации PA13 и PA14*/
    GPIOA->LCKR = GPIO_LCKR_LCKK | GPIO_LCKR_LCK13 | GPIO_LCKR_LCK14;
    GPIOA->LCKR = GPIO_LCKR_LCK13 | GPIO_LCKR_LCK14;
    GPIOA->LCKR = GPIO_LCKR_LCKK | GPIO_LCKR_LCK13 | GPIO_LCKR_LCK14;
    GPIOA->LCKR;
}

/*============================== НАСТРОЙКА RCC =======================================*/
/**
 ***************************************************************************************
 *  @breif Настройка МК STM32F030K6T6 на частоту 48MHz от внешнего кварцевого резонатора
 *  Внешний кварцевый резонатор на 8 MHz
 *  ADC настроен на 12MHz
  *  В настройке также необходимо настроить FLASH на работу, совместимую с 48MHz
 ***************************************************************************************
 */
void CMSIS_RCC_SystemClock_48MHz(void) {
    SET_BIT(RCC->CR, RCC_CR_HSION); //Запустим внутренний RC генератор на 8 МГц
    while (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == 0) ; //Дождемся поднятия флага о готовности
    CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP); //Просто сбросим этот бит в 0(Хотя изначально он и так должен быть в 0).
    SET_BIT(RCC->CR, RCC_CR_HSEON); //Запустим внешний кварцевый резонатор. Он у нас на 8 MHz.
    while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == 0) ; //Дождемся поднятия флага о готовности
    SET_BIT(RCC->CR, RCC_CR_CSSON); //Включим CSS
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSE); //Выберем HSE в качестве System Clock(PLL лучше пока не выбирать, он у нас отключен)
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1); //APB Prescaler /1
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, 0b001 << FLASH_ACR_LATENCY_Pos); //001: One wait state, if 24 MHz < SYSCLK ≤ 48 MHz
    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE); //Prefetch is enabled
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE_Msk, RCC_CFGR_PPRE_DIV1); //APB Prescaler /1
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);
    MODIFY_REG(ADC1->CFGR2, ADC_CFGR2_CKMODE_Msk, 0b10 << ADC_CFGR2_CKMODE_Pos); // 10 : PCLK / 4(Synchronous clock mode)
    SET_BIT(RCC->CFGR, RCC_CFGR_PLLSRC); //В качестве входного сигнала для PLL выберем HSE
    CLEAR_BIT(RCC->CFGR2, 0b0000 << RCC_CFGR2_PREDIV_Pos); //0000: PREDIV input clock not divided
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMUL_Msk, RCC_CFGR_PLLMUL6); //т.к. кварц у нас 8Mhz, а нам нужно 48MHz, то в PLL нужно сделать умножение на 6. 8MHz * 6 = 48MHz.
    SET_BIT(RCC->CR, RCC_CR_PLLON); //Запустим PLL
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL); //Выберем PLL в качестве System Clock
    while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0) ; //Дожидемся поднятия флага включения PLL
}

/**
 ***************************************************************************************
 *  @breif Настройка SysTick на микросекунды
 *  На этом таймере мы настроим Delay и аналог HAL_GetTick()
  ***************************************************************************************
 */

void CMSIS_SysTick_Timer_init(void) {
    CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk); //Выключим таймер для проведения настроек.
    SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk); //Разрешим прерывания по таймеру
    //Прерывание будет происходить каждый раз, когда счетчик отсчитает от заданного значения до 0.
    SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk); //Выберем без делителя. Будет 48MHz
    MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, 47999 << SysTick_LOAD_RELOAD_Pos); //Настроим прерывание на частоту в 1 кГц(т.е. сработка будет каждую мс)
    MODIFY_REG(SysTick->VAL, SysTick_VAL_CURRENT_Msk, 47999 << SysTick_VAL_CURRENT_Pos); //Начнем считать с 47999
    SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk); //Запускаем таймер
    NVIC_EnableIRQ(SysTick_IRQn);
}

/**
 ***************************************************************************************
 *  @breif Настройка Delay и аналог HAL_GetTick()
 ***************************************************************************************
 */

volatile uint32_t SysTimer_ms = 0; //Переменная, аналогичная HAL_GetTick()
volatile uint32_t Delay_counter_ms = 0; //Счетчик для функции Delay_ms
volatile uint32_t Timeout_counter_ms = 0; //Переменная для таймаута функций


/**
 ******************************************************************************
 *  @breif Delay_ms
 *  @param   uint32_t Milliseconds - Длина задержки в миллисекундах
 ******************************************************************************
 */
void Delay_ms(uint32_t Milliseconds) {
    Delay_counter_ms = Milliseconds;
    while (Delay_counter_ms != 0) ;
}

/**
 ******************************************************************************
 *  @breif Прерывание по флагу COUNTFLAG (см. п. 4.5.1 SysTick control and status register (STK_CTRL))
 *  Список векторов(прерываний) можно найти в файле startup_stm32f103c8tx.S
 ******************************************************************************
 */
void SysTick_Handler(void) {

    SysTimer_ms++;

    if (Delay_counter_ms) {
        Delay_counter_ms--;
    }
    if (Timeout_counter_ms) {
        Timeout_counter_ms--;
    }
}

/*================================ Таймеры на примере TIM3 =======================================*/

/**
 ***************************************************************************************
 *  @breif General-purpose timers (TIM2 to TIM5)
 ***************************************************************************************
 */

void CMSIS_TIM3_init(void) {
    /*Включим тактирование таймера (страница 48)*/
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN); //Запуск тактирования таймера 3
    
    //TIMx control register 1 (TIMx_CR1)

    CLEAR_BIT(TIM3->CR1, TIM_CR1_UDIS); //Генерировать событие Update
    CLEAR_BIT(TIM3->CR1, TIM_CR1_URS); //Генерировать прерывание
    CLEAR_BIT(TIM3->CR1, TIM_CR1_OPM); //One pulse mode off(Счетчик не останавливается при обновлении)
    CLEAR_BIT(TIM3->CR1, TIM_CR1_DIR); //Считаем вверх
    MODIFY_REG(TIM3->CR1, TIM_CR1_CMS_Msk, 0b00 << TIM_CR1_CMS_Pos); //Выравнивание по краю
    SET_BIT(TIM3->CR1, TIM_CR1_ARPE); //Auto-reload preload enable
    MODIFY_REG(TIM3->CR1, TIM_CR1_CKD_Msk, 0b00 << TIM_CR1_CKD_Pos); //Предделение выключено

    /*Настройка прерываний*/
    //TIMx DMA/Interrupt enable register (TIMx_DIER)
    CLEAR_BIT(TIM3->DIER, TIM_DIER_UIE); //Update interrupt enable

    //TIMx status register (TIMx_SR) - Статусные регистры

    TIM3->PSC = 24 - 1;
    TIM3->ARR = 1000 - 1;

    // NVIC_EnableIRQ(TIM3_IRQn); //Разрешить прерывания по таймеру 3
    SET_BIT(TIM3->CR1, TIM_CR1_CEN); //Запуск таймера
}

/**
 ******************************************************************************
 *  @breif Прерывание по TIM3
 ******************************************************************************
 */

__WEAK void TIM3_IRQHandler(void) {
    if (READ_BIT(TIM3->SR, TIM_SR_UIF)) {
        CLEAR_BIT(TIM3->SR, TIM_SR_UIF); //Сбросим флаг прерывания
    }
}

//void CMSIS_TIM3_PWM_CHANNEL1_init(void) {
    /*Настройка ножки PA6 под ШИМ*/
  //  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN); //Включим тактирование порта А
    //MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF6_Msk, 0b10 << GPIO_CRL_CNF6_Pos);
    //MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE6_Msk, 0b11 << GPIO_CRL_MODE6_Pos);

    /*Настройка шим(Канал 1)*/
    //MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_CC1S_Msk, 0b00 << TIM_CCMR1_CC1S_Pos); //CC1 channel is configured as output
    //CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC1FE); //Fast mode disable
    //SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC1PE); //Preload enable
    //MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_OC1M_Msk, 0b110 << TIM_CCMR1_OC1M_Pos); //PWM MODE 1
    //CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC1CE); //OC1Ref is not affected by the ETRF input

    /*Запуск ШИМ*/
    //15.4.9 TIMx capture/compare enable register (TIMx_CCER)
    //SET_BIT(TIM3->CCER, TIM_CCER_CC1E); //On - OC1 signal is output on the corresponding output pin. 
    //CLEAR_BIT(TIM3->CCER, TIM_CCER_CC1P); //OC1 active high.

    //TIM3->CCR1 = 0;
//}

void CMSIS_TIM14_init(void) {
    /*Включим тактирование таймера (страница 48)*/
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM14EN); //Запуск тактирования таймера 14
    
    //TIMx control register 1 (TIMx_CR1)

    CLEAR_BIT(TIM14->CR1, TIM_CR1_UDIS); //Генерировать событие Update
    CLEAR_BIT(TIM14->CR1, TIM_CR1_URS); //Генерировать прерывание
    CLEAR_BIT(TIM14->CR1, TIM_CR1_OPM); //One pulse mode off(Счетчик не останавливается при обновлении)
    CLEAR_BIT(TIM14->CR1, TIM_CR1_DIR); //Считаем вниз
    MODIFY_REG(TIM14->CR1, TIM_CR1_CMS_Msk, 0b00 << TIM_CR1_CMS_Pos); //Выравнивание по краю
    SET_BIT(TIM14->CR1, TIM_CR1_ARPE); //Auto-reload preload enable
    MODIFY_REG(TIM14->CR1, TIM_CR1_CKD_Msk, 0b00 << TIM_CR1_CKD_Pos); //Предделение выключено

    /*Настройка прерываний*/
    //TIMx DMA/Interrupt enable register (TIMx_DIER)
    SET_BIT(TIM14->DIER, TIM_DIER_UIE); //Update interrupt enable

    //TIMx status register (TIMx_SR) - Статусные регистры

    TIM14->PSC = 4800 - 1;
    TIM14->ARR = 300 - 1;

    NVIC_SetPriority(TIM14_IRQn, 3);
    NVIC_EnableIRQ(TIM14_IRQn); //Разрешить прерывания по таймеру 14
    SET_BIT(TIM14->CR1, TIM_CR1_CEN); //Запуск таймера
}

/*================================= НАСТРОЙКА ADC ============================================*/

/**
***************************************************************************************
*  @breif Analog-to-digital converter (ADC)
***************************************************************************************
*/

volatile uint16_t ADC_RAW_Data[3] = { 0, }; //Массив, куда будем кидать данные с АЦП

/**
 ******************************************************************************
 *  @breif Настройка ADC1. 3 канала PA0, PA1, PA2 через DMA в буфер ADC_RAW_Data
 ******************************************************************************
 */

void CMSIS_ADC_DMA_init(void) {
    /* Настройка DMA
    *  Внимание:
    *  Порядок настройки DMA хорошо описан на странице 278 "Channel configuration procedure"*/

    SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN); //Включение тактирования DMA1
    DMA1_Channel1->CPAR = (uint32_t) & (ADC1->DR); //Задаем адрес периферийного устройства
    DMA1_Channel1->CMAR = (uint32_t)ADC_RAW_Data; //Задаем адрес в памяти, куда будем кидать данные.
    DMA1_Channel1->CNDTR = 3; //Настроим количество данных для передачи. После каждого периферийного события это значение будет уменьшаться.
    MODIFY_REG(DMA1_Channel1->CCR, DMA_CCR_PL_Msk, 0b00 << DMA_CCR_PL_Pos); //Зададим приоритет канала на высокий
    CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_DIR); //Чтение будем осуществлять с периферии
    SET_BIT(DMA1_Channel1->CCR, DMA_CCR_CIRC); //Настроим DMA в Circular mode
    MODIFY_REG(DMA1_Channel1->CCR, DMA_CCR_PSIZE_Msk, 0b01 << DMA_CCR_PSIZE_Pos); //Размер данных периферийного устройства 16 бит
    MODIFY_REG(DMA1_Channel1->CCR, DMA_CCR_MSIZE_Msk, 0b01 << DMA_CCR_MSIZE_Pos); //Размер данных в памяти 16 бит
    SET_BIT(DMA1_Channel1->CCR, DMA_CCR_TCIE); //Включим прерывание по полной передаче
    CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_HTIE); //Отключим прерывание по половинной передаче
    SET_BIT(DMA1_Channel1->CCR, DMA_CCR_TEIE); //Включим прерывание по ошибке передачи.
    SET_BIT(DMA1_Channel1->CCR, DMA_CCR_MINC); //Включим инкрементирование памяти
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    SET_BIT(DMA1_Channel1->CCR, DMA_CCR_EN); //DMA ON

    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN); //Включение тактирования ADC1.
    SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN); //Включение тактирования порта А.

    /*Настройка ножек PA0 и PA1 на аналоговый вход*/
    /*Pin PA0 - Analog*/
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER0_Msk, 0b11 << GPIO_MODER_MODER0_Pos);
    MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPDR0_Msk, 0b00 << GPIO_PUPDR_PUPDR0_Pos);
    /*Pin PA1 - Analog*/
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER1_Msk, 0b11 << GPIO_MODER_MODER1_Pos);
    MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPDR1_Msk, 0b00 << GPIO_PUPDR_PUPDR1_Pos);
    /*Pin PA2 - Analog*/
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER2_Msk, 0b11 << GPIO_MODER_MODER2_Pos);
    MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPDR2_Msk, 0b00 << GPIO_PUPDR_PUPDR2_Pos);
    
    MODIFY_REG(ADC1->CFGR2, ADC_CFGR2_CKMODE_Msk, 0b10 << ADC_CFGR2_CKMODE_Pos); // 10 : PCLK / 4(Synchronous clock mode)
    
    SET_BIT(ADC1->CFGR1, ADC_CFGR1_DMAEN); //1: DMA enabled
    SET_BIT(ADC1->CFGR1, ADC_CFGR1_DMACFG); //1: DMA circular mode selected
    CLEAR_BIT(ADC1->CFGR1, ADC_CFGR1_SCANDIR); //0: Upward scan (from CHSEL0 to CHSEL17)
    MODIFY_REG(ADC1->CFGR1, ADC_CFGR1_RES_Msk, 0b00 << ADC_CFGR1_RES_Pos); //00: 12 bits
    CLEAR_BIT(ADC1->CFGR1, ADC_CFGR1_ALIGN); //Выравнивание по правому краю
    CLEAR_BIT(ADC1->CFGR1, ADC_CFGR1_OVRMOD); //не перезаписывать ADC DR
    SET_BIT(ADC1->CFGR1, ADC_CFGR1_CONT); //1: Continuous conversion mode
    CLEAR_BIT(ADC1->CFGR1, ADC_CFGR1_WAIT); //0: Wait conversion mode off
    CLEAR_BIT(ADC1->CFGR1, ADC_CFGR1_AUTOFF); //0: Auto-off mode disabled
    CLEAR_BIT(ADC1->CFGR1, ADC_CFGR1_DISCEN); //0: Discontinuous mode disabled
    CLEAR_BIT(ADC1->CFGR1, ADC_CFGR1_AWDEN); //0: Analog watchdog disabled
    CLEAR_BIT(ADC1->CFGR2, ADC_CCR_VREFEN); //0: VREFINT disabled
    CLEAR_BIT(ADC1->CFGR2, ADC_CCR_TSEN); //0: Temperature sensor disabled
    MODIFY_REG(ADC1->SMPR, ADC_SMPR_SMP_Msk, 0b111 << ADC_SMPR_SMP_Pos); //111: 239.5 ADC clock cycles
    SET_BIT(ADC1->CR, ADC_CR_ADCAL); //Enable calibration
    while (READ_BIT(ADC1->CR, ADC_CR_ADCAL)) ;//Подождем окончания калибровки
    Delay_ms(1); 
    SET_BIT(ADC1->CR, ADC_CR_ADEN); //Запустить АЦП
    MODIFY_REG(ADC1->CHSELR, ADC_CHSELR_CHSEL0_Msk, 0b111 << ADC_CHSELR_CHSEL0_Pos);
    SET_BIT(ADC1->CR, ADC_CR_ADSTART); //Запустим преобразования с АЦП
    
}

/**
 ******************************************************************************
 *  @breif Прерывание по DMA1_Channel1
 ******************************************************************************
 */

__WEAK void DMA1_Channel1_IRQHandler(void) {
    if (READ_BIT(DMA1->ISR, DMA_ISR_TCIF1)) {
        SET_BIT(DMA1->IFCR, DMA_IFCR_CGIF1); //Сбросим глобальный флаг.
        /*Здесь можно писать код*/

    }
    else if (READ_BIT(DMA1->ISR, DMA_ISR_TEIF1)) {
        /*Здесь можно сделать какой-то обработчик ошибок*/
        SET_BIT(DMA1->IFCR, DMA_IFCR_CGIF1); //Сбросим глобальный флаг.
    }
}

/*================================= НАСТРОЙКА USART ============================================*/

/**
***************************************************************************************
*  @breif Universal synchronous asynchronous receiver transmitter (USART)
***************************************************************************************
*/

struct USART_name husart1; //Объявляем структуру по USART1

/**
 ******************************************************************************
 *  @breif Настройка USART1. Параметры 9600 8 N 1
 ******************************************************************************
 */

void CMSIS_USART1_Init(void) {

    SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN); //Включение тактирование порта А
    MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL9_Msk, 0b0001 << GPIO_AFRH_AFSEL9_Pos); //USART1_TX AF1
    MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL10_Msk, 0b0001 << GPIO_AFRH_AFSEL10_Pos); //USART1_RX AF1
    
    //Для конфигурирование ножек UART для Full Duplex нужно использовать Alternate function push-pull
    //Tx - PA9
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER9_Msk, 0b10 << GPIO_MODER_MODER9_Pos);
    CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_9);
    MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEEDR9_Msk, 0b11 << GPIO_OSPEEDR_OSPEEDR9_Pos);
    MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPDR9_Msk, 0b00 << GPIO_PUPDR_PUPDR9_Pos);
    //Rx - PA10
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER10_Msk, 0b10 << GPIO_MODER_MODER10_Pos);
    CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_10);
    MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEEDR10_Msk, 0b11 << GPIO_OSPEEDR_OSPEEDR10_Pos);
    MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPDR10_Msk, 0b00 << GPIO_PUPDR_PUPDR10_Pos);
    //Запустим тактирование USART1
    MODIFY_REG(RCC->CFGR3, RCC_CFGR3_USART1SW, RCC_CFGR3_USART1SW_SYSCLK); //Синхронизация USART1 с SYSCLK
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);//Включим тактирование USART1
   
    /*Расчет Fractional baud rate generation
  
    Тогда USARTDIV = 48000000/9600*16 = 312,5
    DIV_Mantissa в данном случае будет 312, что есть 0x138
    DIV_Fraction будет, как 0.5*16 = 12, что есть 0x8

    Тогда весь регистр USART->BRR для скорости 9600 будет выглядеть, как 0x1388.
    */

    MODIFY_REG(USART1->BRR, USART_BRR_DIV_MANTISSA_Msk, 0x138 << USART_BRR_DIV_MANTISSA_Pos);
    MODIFY_REG(USART1->BRR, USART_BRR_DIV_FRACTION_Msk, 0x8 << USART_BRR_DIV_FRACTION_Pos);

    //27.6.4 Control register 1(USART_CR1)(см. стр 821 Reference Manual)
    SET_BIT(USART1->CR1, USART_CR1_UE); //USART enable
    CLEAR_BIT(USART1->CR1, USART_CR1_M); //Word lenght 1 Start bit, 8 Data bits, n Stop bit
    CLEAR_BIT(USART1->CR1, USART_CR1_WAKE); //Wake up idle Line
    CLEAR_BIT(USART1->CR1, USART_CR1_PCE); //Partity control disabled
    //настройка прерываний
    CLEAR_BIT(USART1->CR1, USART_CR1_PEIE); //partity error interrupt disabled
    CLEAR_BIT(USART1->CR1, USART_CR1_TXEIE); //TXE interrupt is inhibited
    CLEAR_BIT(USART1->CR1, USART_CR1_TCIE); //Transmission complete interrupt disabled
    SET_BIT(USART1->CR1, USART_CR1_RXNEIE); //Прерывание по приему данных включено
    SET_BIT(USART1->CR1, USART_CR1_IDLEIE); //Прерывание по флагу IDLE включено
    SET_BIT(USART1->CR1, USART_CR1_TE); //Transmitter is enabled
    SET_BIT(USART1->CR1, USART_CR1_RE); //Receiver is enabled and begins searching for a start bit
    //Остальную настройку, не касающуюся стандартного USART, мы пока трогать не будем, но на всякий случай обнулим
    USART1->CR2 = 0;
    CLEAR_BIT(USART1->CR2, USART_CR2_STOP); //1 стоп бит.
    USART1->CR3 = 0;
    USART1->GTPR = 0;

    NVIC_EnableIRQ(USART1_IRQn); //Включим прерывания по USART1
}

/**
 ******************************************************************************
 *  @breif Прерывание по USART1
 ******************************************************************************
 */

__WEAK void USART1_IRQHandler(void) {
    if (READ_BIT(USART1->ISR, USART_ISR_RXNE)) {
        //Если пришли данные по USART
        husart1.rx_buffer[husart1.rx_counter] = USART1->RDR; //Считаем данные в соответствующую ячейку в rx_buffer
        husart1.rx_counter++; //Увеличим счетчик принятых байт на 1
    }
    if (READ_BIT(USART1->ISR, USART_ISR_IDLE)) {
        //Если прилетел флаг IDLE
        SET_BIT(USART1->ICR, USART_ICR_IDLECF); //Сбросим флаг IDLE
        husart1.rx_len = husart1.rx_counter; //Узнаем, сколько байт получили
        husart1.rx_counter = 0; //сбросим счетчик приходящих данных
    }
}

/**
 ******************************************************************************
 *  @breif Функция отправки данных по USART
 *  @param  *USART - USART, с которого будут отправляться данные
 *  @param  *data - данные, которые будем отправлять
 *  @param  Size - сколько байт требуется передать
 ******************************************************************************
 */

bool CMSIS_USART_Transmit(USART_TypeDef* USART, uint8_t* data, uint16_t Size, uint32_t Timeout_ms) {
    for (uint16_t i = 0; i < Size; i++) {
        Timeout_counter_ms = Timeout_ms;
        //Ждем, пока линия не освободится
        while (READ_BIT(USART->ISR, USART_ISR_TXE) == 0) {
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        USART->TDR = *data++; //Кидаем данные  
    }
    return true;
}

/*================================= Работа с FLASH ============================================*/

/*Пример структуры для работы с FLASH*/
/*
typedef struct __attribute__((packed)) {
    uint8_t Data1;
    uint16_t Data2;
    uint32_t Data3;
    float Data4;
} Flash_struct;

Flash_struct Flash_data_STM;
Flash_struct Flash_data_STM1;*/

/*Пример работы с FLASH*/
/*
Flash_data_STM.Data1 = 0x23;
Flash_data_STM.Data2 = 0x4567;
Flash_data_STM.Data3 = 0x89101112;
Flash_data_STM.Data4 = 3.14159f;
FLASH_Page_write(0x08007C00, (uint8_t*)&Flash_data_STM, sizeof(Flash_data_STM));
FLASH_Read_data(0x08007C00, (uint8_t*)&Flash_data_STM1, sizeof(Flash_data_STM1));
*/
        

/**
 ***************************************************************************************
 *  @breif Разблокировка FLASH
 *  Чтоб разблокировать FLASH, нужно в FLASH->KEYR ввести поочередно 2 ключа.
  ***************************************************************************************
 */
void FLASH_Unlock(void) {
    FLASH->KEYR = 0x45670123; //KEY1
    FLASH->KEYR = 0xCDEF89AB; //KEY2
}

/**
 ***************************************************************************************
 *  @breif Блокировка FLASH
 *  Чтоб заблокировать FLASH, нужно в FLASH->CR, FLASH_CR_LOCK выставить 1
  ***************************************************************************************
 */
void FLASH_Lock(void) {
    SET_BIT(FLASH->CR, FLASH_CR_LOCK);
}

/**
 ***************************************************************************************
 *  @breif Стирание страницы во FLASH
 *  @param  Adress - Адрес во flash
 ***************************************************************************************
 */
void FLASH_Page_erase(uint16_t Adress) {
    //Если память заблокирована, то разблокируем ее
    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK)) {
        FLASH_Unlock();
    }
    SET_BIT(FLASH->CR, FLASH_CR_PER); //Выберем функцию очистки страницы
    FLASH->AR = Adress; //Укажем адрес
    SET_BIT(FLASH->CR, FLASH_CR_STRT); //Запустим стирание
    while (READ_BIT(FLASH->SR, FLASH_SR_BSY)) ; //Ожидаем, пока пройдет стирание
    while (READ_BIT(FLASH->SR, FLASH_SR_EOP) == 0) ; //Дождемся флага завершения программы
    CLEAR_BIT(FLASH->CR, FLASH_CR_PER); //Выключим функцию.
    FLASH_Lock(); //Заблокируем память
}

/**
 ***************************************************************************************
 *  @breif Запись страницы во FLASH
 *  Programming Manual PM0075
 *  см. п.п. 2.3.3 Main Flash memory programming(стр. 13)
 *  @param  Adress - Адрес во flash
 *  @param  *Data - Данные, которые будем писать во flash
 *  @param  Size - Размер даных, которые будем писать во flash
 ***************************************************************************************
 */
void FLASH_Page_write(uint32_t Adress, uint8_t *Data, uint16_t Size) {
    //Проверка размера данных на четность
    //Если размер нечетный
    if (Size % 2) {
        Size = (Size / 2); //Размер в Half-word
        FLASH_Page_erase(Adress); //Произведем стирание страницы
        //Если память заблокирована, то разблокируем ее
        if (READ_BIT(FLASH->CR, FLASH_CR_LOCK)) {
            FLASH_Unlock();
        }
        SET_BIT(FLASH->CR, FLASH_CR_PG); //Выберем программу "programming"
        //Заполним ячейки по 16 бит
        for (int i = 0; i < Size; i++) {
            *(uint16_t*)(Adress + i * 2) = *((uint16_t*)(Data) + i);
        }
        //Заполним остаток в 8 бит
        *(uint16_t*)(Adress + Size * 2) = *((uint8_t*)(Data) + Size * 2);
    }
    //Если размер четный
    else {
        Size = (Size / 2); //Размер в Half-word
        FLASH_Page_erase(Adress); //Произведем стирание страницы
        //Если память заблокирована, то разблокируем ее
        if (READ_BIT(FLASH->CR, FLASH_CR_LOCK)) {
            FLASH_Unlock();
        }
        SET_BIT(FLASH->CR, FLASH_CR_PG); //Выберем программу "programming"
        //Заполним ячейки по 16 бит
        for (int i = 0; i < Size; i++) {
            *(uint16_t*)(Adress + i * 2) = *((uint16_t*)(Data) + i);
        }
    }
    while (READ_BIT(FLASH->SR, FLASH_SR_BSY)) ;
    while (READ_BIT(FLASH->SR, FLASH_SR_EOP) == 0) ;
    CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
    FLASH_Lock(); 
}

/**
 ***************************************************************************************
 *  @breif Считывание данных с FLASH. 
 *  Programming Manual PM0075
 *  см. п.п. 2.3.3 Main Flash memory programming(стр. 13)
 *  @param  Adress - Адрес во flash, откуда будем забирать данные
 *  @param  *Data - Данные, куда будем записывать информацию из flash с указанного адреса
 *  @param  Size - Размер даных. Сколько байт будем считывать.
 ***************************************************************************************
 */
void FLASH_Read_data(uint32_t Adress, uint8_t *Data, uint16_t Size) {
    //Проверка размера данных на четность
    //Если размер структуры в байтах нечетный
    if (Size % 2) {
        Size = (Size / 2); //Размер в Half-word
        //Считаем данные по 16 бит
        for (uint16_t i = 0; i < Size; i++) {
            *((uint16_t*)Data + i) = *(uint16_t*)(Adress + i * 2);
        }
        //Считаем оставшиеся 8 бит
        *((uint8_t*)Data + Size * 2) = *(uint16_t*)(Adress + Size * 2);
    }//Если размер структуры в байтах четный
    else {
        Size = (Size / 2); //Размер в Half-word
        //Считаем информацию по 16 бит
        for (uint16_t i = 0; i < Size; i++) {
            *((uint16_t*)Data + i) = *(uint16_t*)(Adress + i * 2);
        }
    }
}
