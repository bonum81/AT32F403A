/**
  **************************************************************************
  * @file     app.cpp
  * @brief    test program for at32f403AVGT
  **************************************************************************
  */

/* includes ------------------------------------------------------------------*/

#include "at32f403a_407.h"

volatile uint32_t ticks_delay = 0;
uint8_t mode_blink = 0;

void system_clock_config(void);
void SysTickTimerInit();
void ledOnOff(uint32_t milliseconds, gpio_type *gpio_x, uint16_t pins);
void delay(const uint32_t milliseconds);


int main(void)
{
    
    gpio_init_type gpio_leds;
    gpio_init_type gpio_button;

    system_clock_config();

    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

    gpio_leds.gpio_pins = GPIO_PINS_12 | GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15;
    gpio_leds.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_leds.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_leds.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_leds.gpio_pull = GPIO_PULL_NONE;

    gpio_init(GPIOD, &gpio_leds);

    gpio_button.gpio_pins = GPIO_PINS_0;
    gpio_button.gpio_mode = GPIO_MODE_INPUT;
    gpio_button.gpio_pull = GPIO_PULL_DOWN;

    gpio_init(GPIOA, &gpio_button);

    SysTickTimerInit();

    while (1)
    {
        /* code */
        //ledOnOff(100, GPIOD, GPIO_PINS_12);
        //ledOnOff(100, GPIOD, GPIO_PINS_13);


        if (gpio_input_data_bit_read(GPIOA, GPIO_PINS_0) == SET)
        {
             delay(100);
            mode_blink++;

            if(mode_blink > 3)
            {
                mode_blink = 0;
            }
        }

        switch (mode_blink)
        {
        case 1:
            /* code */
            ledOnOff(100, GPIOD, GPIO_PINS_14);
            ledOnOff(100, GPIOD, GPIO_PINS_15);
            ledOnOff(100, GPIOD, GPIO_PINS_12);
            ledOnOff(100, GPIOD, GPIO_PINS_13);
            break;
        case 2:
            gpio_bits_set(GPIOD, GPIO_PINS_12);
            gpio_bits_set(GPIOD, GPIO_PINS_13);
            gpio_bits_set(GPIOD, GPIO_PINS_14);
            gpio_bits_set(GPIOD, GPIO_PINS_15);
            break;
        case 3:
            gpio_bits_set(GPIOD, GPIO_PINS_12);
            gpio_bits_set(GPIOD, GPIO_PINS_13);
            gpio_bits_reset(GPIOD, GPIO_PINS_14);
            gpio_bits_reset(GPIOD, GPIO_PINS_15);
         break;

        default:
            gpio_bits_reset(GPIOD, GPIO_PINS_12);
            gpio_bits_reset(GPIOD, GPIO_PINS_13);
            gpio_bits_reset(GPIOD, GPIO_PINS_14);
            gpio_bits_reset(GPIOD, GPIO_PINS_15);
            break;
        }
       
    }
    
}

void ledOnOff(uint32_t milliseconds, gpio_type *gpio_x, uint16_t pins)
{
    gpio_bits_set(gpio_x, pins);
    delay(milliseconds);
    gpio_bits_reset(gpio_x, pins);
    delay(milliseconds);
}

/**
  * @brief  system clock config program
  * @note   the system clock is configured as follow:
  *         - system clock        = hext * pll_mult
  *         - system clock source = pll (hext)
  *         - hext                = 8000000
  *         - sclk                = 80000000
  *         - ahbdiv              = 1
  *         - ahbclk              = 80000000
  *         - apb1div             = 2
  *         - apb1clk             = 40000000
  *         - apb2div             = 2
  *         - apb2clk             = 40000000
  *         - pll_mult            = 10
  *         - pll_range           = GT72MHZ (greater than 72 mhz)
  * @param  none
  * @retval none
  */
void system_clock_config(void)
{
  /* Сбрасываем crm регистр */
  crm_reset();                          

  /* Включаем внешний кварц */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);

  /* Ждем включения внешнего кварца */
  while(crm_hext_stable_wait() == ERROR)
  {
  }

  /* Выбираем источник тактов для PLL, выставляем множитель, диапазон получаемой частоты */
  crm_pll_config(CRM_PLL_SOURCE_HEXT, CRM_PLL_MULT_10, CRM_PLL_OUTPUT_RANGE_GT72MHZ);

  /* Включаем PLL */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);

  /* Ждем включения PLL */
  while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET)
  {
  }

  /* Делитель для ahbclk */
  crm_ahb_div_set(CRM_AHB_DIV_1);

  /* Делитель для apb2clk */
  crm_apb2_div_set(CRM_APB2_DIV_2);

  /* Делитель для apb1clk */
  crm_apb1_div_set(CRM_APB1_DIV_2);

  /* Выбираем PLL как источник системной частоты */
  crm_sysclk_switch(CRM_SCLK_PLL);

  /* Ждем переключения системной частоты на PLL */
  while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL)
  {
  }

  /* Обновляем глобальные переменные с системной частотой */
  system_core_clock_update();
}

void delay(const uint32_t milliseconds) 
{

    uint32_t start = ticks_delay;

    while((ticks_delay - start) < milliseconds);

}

void SysTickTimerInit()
{
    SysTick->LOAD = 80000-1;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

    NVIC_EnableIRQ(SysTick_IRQn);
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}




#if defined(__cplusplus)
extern "C"{
#endif  

void SysTick_Handler(void)
{
    ticks_delay++;
}



#if defined(__cplusplus)
}
#endif
