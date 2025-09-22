/*
 * adc_read.c
 *
 *  Created on: 2022. 3. 27.
 *      Author: dooker
 */

/*===========================================================================

                       INCLUDE FILES FOR MODULE

===========================================================================*/
#include "app.h"
/*===========================================================================

                DEFINITIONS AND DECLARATIONS FOR MODULE

This section contains definitions for constants, macros, types, variables
and other items needed by this module.

===========================================================================*/

/*===========================================================================

                         DEFINE DEFINITIONS

===========================================================================*/

#define IADC_12BIT_MODE      0
#define IADC_16BIT_MODE      1
#define IADC_MODE    IADC_16BIT_MODE


#if (IADC_MODE==IADC_12BIT_MODE)
// Set CLK_ADC to 10kHz (this corresponds to a sample rate of 1ksps)
#define CLK_SRC_ADC_FREQ        5000000  // CLK_SRC_ADC; largest division is by 4
#define CLK_ADC_FREQ                  10000       // CLK_ADC; IADC_SCHEDx PRESCALE has 10 valid bits
#else
// Set CLK_ADC to 10MHz (this corresponds to a sample rate of 77K with OSR = 32)
#define CLK_SRC_ADC_FREQ         20000000     // CLK_SRC_ADC; largest division is by 4
#define CLK_ADC_FREQ                  10000000     // CLK_ADC; IADC_SCHEDx PRESCALE has 10 valid bits
#endif


/*===========================================================================

                        LOCAL DEFINITIONS

===========================================================================*/


/*===========================================================================

                        GLOBAL VARIABLE

===========================================================================*/

/*===========================================================================

                        FUNCTION PROTOTYPES

===========================================================================*/


//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//  adc polling sequence...
//
static void IADC_disable(IADC_TypeDef *iadc)
{
#if defined(IADC_STATUS_SYNCBUSY)
  while ((iadc->STATUS & IADC_STATUS_SYNCBUSY) != 0U)
  {
    // Wait for synchronization to finish before disable
  }
#endif
  iadc->EN_CLR = IADC_EN_EN;
}

static void IADC_enable(IADC_TypeDef *iadc)
{
  iadc->EN_SET = IADC_EN_EN;
}


/**************************************************************************//**
 * @brief  Initialize IADC function
 *****************************************************************************/
void IADC_Polling_Init(void)
{
  // Declare init structs
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;

  IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
  IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

  // Enable IADC0 and GPIO clock branches
  CMU_ClockEnable(cmuClock_IADC0, true);

  // Reset IADC to reset configuration in case it has been modified by
  // other code
  IADC_reset(IADC0);

  // Select clock for IADC
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);  // FSRCO - 20MHz

  // Modify init structs and initialize
  init.warmup = iadcWarmupKeepWarm;

  // Set the HFSCLK prescale value here
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);          // 10000000 // CLK_SRC_ADC

  // Configuration 0 is used by both scan and single conversions by default
  // Use unbuffered AVDD as reference
  initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2;
  initAllConfigs.configs[0].analogGain= iadcCfgAnalogGain0P5x;

  // Divides CLK_SRC_ADC to set the CLK_ADC frequency
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                       CLK_ADC_FREQ,
                       0,
                       iadcCfgModeNormal,
                       init.srcClkPrescale);

  // Set oversampling rate to 32x
  // resolution formula res = 11 + log2(oversampling * digital averaging)
  // in this case res = 11 + log2(32*1) = 16
  initAllConfigs.configs[0].osrHighSpeed = iadcCfgOsrHighSpeed32x;

  // Assign pins to positive and negative inputs in differential mode

  // Initialize the IADC
  IADC_init(IADC0, &init, &initAllConfigs);

  // Initialize the Single conversion inputs
  IADC_initSingle(IADC0, &initSingle, &initSingleInput);

  IADC_disable(IADC0);
}

/**************************************************************************//**
 * @brief  Read ADC data
 *****************************************************************************/
float read_batt_lvl_mV(void)
{
    uint32_t adc_single_result;
    double batt_lvl;

    IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
    IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

#if (IADC_MODE==IADC_16BIT_MODE)
  // Single initialization
  initSingle.dataValidLevel = iadcFifoCfgDvl1;

  // Set alignment to right justified with 16 bits for data field
  initSingle.alignment = iadcAlignRight16;
#endif

  // Assign pins to positive and negative inputs in differential mode
  initSingleInput.posInput   = iadcPosInputAvdd;
  initSingleInput.negInput   = iadcNegInputGnd;
  IADC_initSingle(IADC0, &initSingle, &initSingleInput);

  IADC_enable(IADC0);

  // Start IADC conversion
  IADC_command(IADC0, iadcCmdStartSingle);

  // Wait for conversion to be complete
  while((IADC0->STATUS & (_IADC_STATUS_CONVERTING_MASK
        | _IADC_STATUS_SINGLEFIFODV_MASK)) != IADC_STATUS_SINGLEFIFODV); //while combined status bits 8 & 6 don't equal 1 and 0 respectively

  // Get ADC result
  adc_single_result = IADC_pullSingleFifoResult(IADC0).data;

  // Calculate input voltage:
  // For single-ended the result range is 0 to +Vref, i.e., 12 bits for the conversion value.
  // The result of AVDD measurement is attenuated by a factor of 4
  // Therefore in order to get the full ADD measurement, the result needs to be multiplied by 4
//  batt_lvl = (adc_single_result * 1.21 * 4) / 0xFFF;
#if (IADC_MODE==IADC_12BIT_MODE)
  batt_lvl = (adc_single_result * 2.42 * 4) / 0xFFF;
#else
  batt_lvl = (adc_single_result * 2.42 * 4) / 0xFFFF;
#endif

  IADC_disable(IADC0);

  batt_lvl+=0.045;

  return batt_lvl;
}








