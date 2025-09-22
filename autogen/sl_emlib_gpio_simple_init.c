#include "sl_emlib_gpio_simple_init.h"
#include "sl_emlib_gpio_init_latch_addr_0_config.h"
#include "sl_emlib_gpio_init_latch_addr_1_config.h"
#include "sl_emlib_gpio_init_latch_addr_2_config.h"
#include "sl_emlib_gpio_init_led_pin_config.h"
#include "sl_emlib_gpio_init_sens_power_pin_config.h"
#include "em_gpio.h"
#include "em_cmu.h"

void sl_emlib_gpio_simple_init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_PORT,
                  SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_PIN,
                  SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_MODE,
                  SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_PORT,
                  SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_PIN,
                  SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_MODE,
                  SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_PORT,
                  SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_PIN,
                  SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_MODE,
                  SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_LED_PIN_PORT,
                  SL_EMLIB_GPIO_INIT_LED_PIN_PIN,
                  SL_EMLIB_GPIO_INIT_LED_PIN_MODE,
                  SL_EMLIB_GPIO_INIT_LED_PIN_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_SENS_POWER_PIN_PORT,
                  SL_EMLIB_GPIO_INIT_SENS_POWER_PIN_PIN,
                  SL_EMLIB_GPIO_INIT_SENS_POWER_PIN_MODE,
                  SL_EMLIB_GPIO_INIT_SENS_POWER_PIN_DOUT);
}
