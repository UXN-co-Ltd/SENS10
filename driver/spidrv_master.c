/***************************************************************************//**
 * @file
 * @brief spidrv master micriumos examples functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include <math.h>

#include "spidrv.h"
#include "sl_spidrv_instances.h"
#include "sl_spidrv_usart_spi_sens_config.h"
#include "sl_emlib_gpio_init_sens_power_pin_config.h"
#include "sl_emlib_gpio_init_latch_addr_0_config.h"
#include "sl_emlib_gpio_init_latch_addr_1_config.h"
#include "sl_emlib_gpio_init_latch_addr_2_config.h"
#include "sl_emlib_gpio_init_led_pin_config.h"

#include "app.h"
#include "ustimer.h"

#include "histrogram_table.h"

#define _DEBUG_MSG_   0

#define LOG_FLOAT_MARKER "%c%d.%04d"
#define LOG_FLOAT(val) (val > 0) ?' ':'-', \
             (int32_t)(val),     \
                       (int32_t)(((val > 0) ? (val) - (int32_t)(val)       \
                       : (int32_t)(val) - (val))*10000)



#define POLYNOMIAL    0xA001              // Generator polynomial for CRC
#define InitCRCval          0xFFFF               // Initial CRC value

// use SPI handle for EXP header (configured in project settings)
#define SPI_HANDLE                  sl_spidrv_usart_spi_sens_handle

// size of transmission and reception buffers
#define APP_BUFFER_SIZE             64

static sl_sleeptimer_timer_handle_t spidrv_transfer_timer;
#define SPIDRV_TRANSFER_INTERVAL_MS                    (1000)


unsigned char SPI_in[200];
unsigned char SPI_in_index;

unsigned char OPC_R2_buff[512+8];

volatile int loop_timer = 0;

bool pan_state = 0;
bool pan_power_state = 0;

config_variables_t config_variables;
Histogram_data_t  Histogram_data;
PM_data_t               PM_data;

OPC_R2_info_t       OPC_R2_info;


#define SET_OPC_R2_ALL_STATE(val)     OPC_R2_info.sens_state.bAll=val
#define GET_OPC_R2_ALL_STATE()          OPC_R2_info.sens_state.bAll


int Histrogram_sample_pos = 0;

/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/

/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/
/*******************************************************************************
 * Initialize example.
 ******************************************************************************/
//===========================================================================================
//  AVDD POWER ENABLE & DISABLE
//===========================================================================================
void sens_power_control(int8_t on_off)
{
    if(on_off)
        GPIO_PinOutSet(SL_EMLIB_GPIO_INIT_SENS_POWER_PIN_PORT, SL_EMLIB_GPIO_INIT_SENS_POWER_PIN_PIN);
    else
        GPIO_PinOutClear(SL_EMLIB_GPIO_INIT_SENS_POWER_PIN_PORT, SL_EMLIB_GPIO_INIT_SENS_POWER_PIN_PIN);
}

void sens_led_control(int8_t on_off)
{
    if(on_off)
        GPIO_PinOutSet(SL_EMLIB_GPIO_INIT_LED_PIN_PORT, SL_EMLIB_GPIO_INIT_LED_PIN_PIN);
    else
        GPIO_PinOutClear(SL_EMLIB_GPIO_INIT_LED_PIN_PORT, SL_EMLIB_GPIO_INIT_LED_PIN_PIN);
}


void set_OPC_R2_latch_address(uint8_t address)
{
    switch(address) {
        case OPC_R2_NUM_1:   // 0 0 0
            GPIO_PinOutClear(SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_PIN);
            GPIO_PinOutClear(SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_PIN);
            GPIO_PinOutClear(SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_PIN);
            break;

        case OPC_R2_NUM_2:     // 1 0 0
                GPIO_PinOutSet(SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_PIN);
                GPIO_PinOutClear(SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_PIN);
                GPIO_PinOutClear(SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_PIN);
                break;

        case OPC_R2_NUM_3:   // 0 1 0
            GPIO_PinOutClear(SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_PIN);
            GPIO_PinOutSet(SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_PIN);
            GPIO_PinOutClear(SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_PIN);
            break;

        case OPC_R2_NUM_4:   // 1 1 0
            GPIO_PinOutSet(SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_PIN);
            GPIO_PinOutSet(SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_PIN);
            GPIO_PinOutClear(SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_PIN);
            break;

        case OPC_R2_NUM_5:   // 1 1 1
            GPIO_PinOutSet(SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_PIN);
            GPIO_PinOutSet(SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_PIN);
            GPIO_PinOutSet(SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_PIN);
            break;

        case OPC_R2_NUM_6:   // 0 1 1
            GPIO_PinOutClear(SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_PIN);
            GPIO_PinOutSet(SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_PIN);
            GPIO_PinOutSet(SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_PIN);
            break;

        case OPC_R2_NUM_7:   // 1 0 1
            GPIO_PinOutSet(SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_PIN);
            GPIO_PinOutClear(SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_PIN);
            GPIO_PinOutSet(SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_PIN);
            break;

        case OPC_R2_NUM_8:   // 0 0 1
            GPIO_PinOutClear(SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_0_PIN);
            GPIO_PinOutClear(SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_1_PIN);
            GPIO_PinOutSet(SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_PORT, SL_EMLIB_GPIO_INIT_LATCH_ADDR_2_PIN);
            break;

        default:
            break;
    }

    delay_ms(1);
}

void delay_us(uint32_t usec)
{
    USTIMER_DelayIntSafe(usec);
}

void spidrv_app_init(void)
{
    uint8_t i;

  USTIMER_Init();

    sens_power_control(POWER_OFF);
    delay_ms(100);
    sens_power_control(POWER_ON);
    delay_ms(100);

//  OPC-R2 RESET
    for(i=0;i<8;i++) {
        set_OPC_R2_latch_address(i);
        set_OPC_R2_reset();
        delay_ms(10);
    }

    OPC_R2_info.device_step =  OPC_R2_DEVICE_INIT;
//    OPC_R2_info.device_step =  OPC_R2_READY;



    OPC_R2_info.latch_pos = 0;
    OPC_R2_info.ini_count = 0;

    SET_OPC_R2_ALL_STATE(0x00);

    run_spidrv_transfer_timer();


    OPC_R2_info.device_step =  OPC_R2_SIMULATION;
    SET_OPC_R2_ALL_STATE(0xFF);


}


//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  CS LOW ACTIVE -> CS HIGH DE-ACTIVE
//

/***********************************************************************************************************************
  *   UTILITY FUNCTION
  ***********************************************************************************************************************/
void make_ltoa(uint8_t  *au8buff, uint32_t val)
{
    au8buff[0]=(unsigned char)((val>>16)&0xFF);         // H
    au8buff[1]=(unsigned char)((val>>8)&0xFF);            // M
    au8buff[2]=(unsigned char)(val&0xFF);                       // L
}

uint32_t make_atol(uint8_t * acBuff, uint8_t i)
{
  uint32_t retVal=0;

    retVal=(uint32_t)(acBuff[i]&0xFF);
    retVal=retVal<<8;
    retVal|=(uint32_t)(acBuff[i+1]&0xFF);
    retVal=retVal<<8;
    retVal|=(uint32_t)(acBuff[i+2]&0xFF);

    return retVal;
}

void spidrv_cs_enable(void)
{
    GPIO_PinModeSet(SL_SPIDRV_USART_SPI_SENS_CS_PORT, SL_SPIDRV_USART_SPI_SENS_CS_PIN, gpioModePushPull, 1);
    GPIO_PinOutClear(SL_SPIDRV_USART_SPI_SENS_CS_PORT, SL_SPIDRV_USART_SPI_SENS_CS_PIN);
}


void spidrv_cs_disable(void)
{
    GPIO_PinModeSet(SL_SPIDRV_USART_SPI_SENS_CS_PORT, SL_SPIDRV_USART_SPI_SENS_CS_PIN, gpioModePushPull, 1);
    GPIO_PinOutSet(SL_SPIDRV_USART_SPI_SENS_CS_PORT, SL_SPIDRV_USART_SPI_SENS_CS_PIN);
}


/* --------------------------------------------------------------------------------------
  *  Functions definitions
  * -------------------------------------------------------------------------------------- */
int spidrv_read_reg_action( uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
  Ecode_t retVal = ECODE_EMDRV_SPIDRV_OK;
  uint8_t com_buffer[APP_BUFFER_SIZE];
  uint8_t i;

  com_buffer[0] = reg;
  com_buffer[1] = 0x80;

  for(i=0;i<rlen;i++) {
      com_buffer[i+2] = 0xAA;       // dummy data clock
  }

//  spidrv_cs_enable();
  retVal = SPIDRV_MTransferB(sl_spidrv_usart_spi_sens_handle, com_buffer, com_buffer, rlen+2);
//  spidrv_cs_disable();


  memcpy(rbuffer, &com_buffer[2], rlen);

  if(retVal != ECODE_EMDRV_SPIDRV_OK)
  {
      printf("Error to read SPI(0x%lX)", retVal);
      return false;
  }

  return true;
}

int spidrv_write_reg_action(  uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
  Ecode_t retVal = ECODE_EMDRV_SPIDRV_OK;
  uint8_t com_buffer[APP_BUFFER_SIZE];

  com_buffer[0] = reg;
  com_buffer[1] = 0x00;
  memcpy(&com_buffer[2], wbuffer, wlen);

  spidrv_cs_enable();
  retVal = SPIDRV_MTransferB(sl_spidrv_usart_spi_sens_handle, com_buffer, com_buffer, wlen+2);
  spidrv_cs_disable();

  if(retVal != ECODE_EMDRV_SPIDRV_OK)
  {
      printf( "Error to write SPI(0x%lX)", retVal);
    return false;
  }

  return true;
}


/**************************************************************************************************************************
  *
  *   Below OPC-R2 Utility Function
  *
  **************************************************************************************************************************/
uint16_t make_16bit(uint8_t LSB, uint8_t MSB) {
    return ((MSB << 8) | LSB);
}


uint32_t make_32bit(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
    return (uint32_t)((b3 << 24) | (b2 << 16) | (b1 << 8) | b0);
}


/**************************************************************************************************************************
  *   Convert SHT31 ST output to Temperature (C)
  **************************************************************************************************************************/
float make_Temperature (uint16_t ST)
{
    return -45 + 175*(float)ST/65535;
}

/**************************************************************************************************************************
  *   Convert SHT31 SRH output to Relative Humidity (%)
  **************************************************************************************************************************/
float make_RelativeHumidity (uint16_t SRH)
{
    return 100*(float)SRH/65535;
}


/**************************************************************************************************************************
  *   Convert SHT31 SRH output to Relative Humidity (%)
  **************************************************************************************************************************/
uint16_t CalcCRC(uint8_t data[], uint8_t nbrOfBytes)
{
    uint8_t _bit;                             // bit mask
    uint16_t crc = InitCRCval;    // initialise calculated checksum
    uint8_t byteCtr;                       // byte counter

// calculates 16-Bit checksum with given polynomial
    for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
        crc ^= (uint16_t)data[byteCtr];

        for(_bit = 0; _bit < 8; _bit++)        {
            if (crc & 1)  {             //if bit0 of crc is 1
                crc >>= 1;
                crc ^= POLYNOMIAL;
            }
            else {
              crc >>= 1;
            }
        }
    }

    return crc;
}

/**************************************************************************************************************************
  *   OPC-R2 SPI Transfer code
  **************************************************************************************************************************/
uint8_t spidrv_transfer(uint8_t command)
{
    Ecode_t retVal = ECODE_EMDRV_SPIDRV_OK;
    uint8_t com_buffer[4];

    com_buffer[0] = command;
    retVal = SPIDRV_MTransferB(sl_spidrv_usart_spi_sens_handle, com_buffer, com_buffer, 1);

    if(retVal != ECODE_EMDRV_SPIDRV_OK)  {
        printf("Error to read SPI(0x%lX)", retVal);
        return 0xff;
    }

    return com_buffer[0];
}



/**************************************************************************************************************************
  *   OPC-R2 Discard remaining bytes in SPI buffer
  **************************************************************************************************************************/
void spidrv_discard_buffer (uint8_t NumToDiscard)
{
    for (SPI_in_index=0; SPI_in_index<NumToDiscard; ++SPI_in_index) {
        delay_us(10);
        spidrv_transfer(0x01); //Send dummy byte to OPC
    }
}

/**************************************************************************************************************************
  *   OPC-R2 command ready state check
  **************************************************************************************************************************/
bool spidrv_opc_command_ready(uint8_t command)
{
    uint8_t Response = 0;
    uint8_t Tries = 0;

    do {
        Response = spidrv_transfer(command);
        if(Response!=OPC_CMD_READY)
            delay_us(10);
    }   while ((Tries++ < OPC_MAX_TRIES) && (Response != OPC_CMD_READY));

    delay_ms(10);

    if(Tries>OPC_MAX_TRIES) {
#if _DEBUG_MSG_
        printf("opc command false\r\n");
#endif
        return false;
    }

  return true;
}

/***************************************************************************************************************************
  * read Firmware_version
  **************************************************************************************************************************/
bool set_OPC_R2_reset(void)
{
    if(spidrv_opc_command_ready(OPC_CMD_RESET)==false) {
        return false;
    }

    return true;
}


/***************************************************************************************************************************
  * read Firmware_version
  **************************************************************************************************************************/
bool read_Firmware_Version(void)
{
    uint8_t i;
    uint8_t data_In[8];
    firmwareVersion data_Out;

    if(spidrv_opc_command_ready(OPC_CMD_READ_FW_VERSION)==false)
        return false;

    for (i = 0 ; i < 2 ;  i++)  {
        delay_us(10);
        data_In[i] = spidrv_transfer(0x00);
    }

    memcpy(&data_Out, &data_In, sizeof(data_Out));
    printf("%d.%d\t%d.%d\r\n", data_In[0], data_In[1], data_Out.major, data_Out.minor);

    return true;
}

/***************************************************************************************************************************
  * read Serial Number
  **************************************************************************************************************************/
bool read_Serial_Number(void)
{
    uint8_t i;
    uint8_t data_In[64];

    if(spidrv_opc_command_ready(OPC_CMD_READ_SERIAL_STRING)==false)
        return false;

#if _DEBUG_MSG_
    printf("serial number : ");
#endif

    for (i = 0 ; i < 60 ;  i++)  {
        delay_us(10);
        data_In[i] = spidrv_transfer(0x00);
        printf("[%c] ", data_In[i]);
    }
    printf("\r\n");


    return true;
}

/***************************************************************************************************************************
  * read Information
  **************************************************************************************************************************/
bool read_Information(void)
{
    uint8_t i;
    uint8_t data_In[64];

    if(spidrv_opc_command_ready(OPC_CMD_READ_INFO_STRING)==false) {
        return false;
    }

    memset(data_In, 0, sizeof(data_In));
    for (i = 0 ; i < 60 ;  i++)  {
        delay_us(10);
        data_In[i] = spidrv_transfer(0x00);
    }

    if (strncmp(data_In, "OPC-R2", 6) == 0) {
#if _DEBUG_MSG_
        printf("read_Information : %s\r\n", data_In);
#endif
        return true;
    }

    return false;
}

/***************************************************************************************************************************
  * read Configuration...max 193...data read
  **************************************************************************************************************************/
bool read_Configuration(void)
{
    uint8_t i;
    uint8_t ps;
    uint8_t QPos;

    uint16_t *pWord;
    float       *pFloat;

    if(spidrv_opc_command_ready(OPC_CMD_READ_CONFIG)==false)
        return false;

    memset(SPI_in, 0, sizeof(SPI_in));
    for (i = 0 ; i < 196 ;  i++)  {
        delay_us(10);
        SPI_in[i] = spidrv_transfer(0x00);
    }

//--------------------------------------------------------------------------------------------------------------------------------
//  below parser configuration
//--------------------------------------------------------------------------------------------------------------------------------
    QPos = ps = 0;
    for(i=0 ; i<34 ; i+=2) {
        pWord = (uint16_t *)&SPI_in[i];
        config_variables.binBoundriesADC[ps++] = *pWord;
    }
    QPos += 34;

#if _DEBUG_MSG_
    printf("binBoundriesADC : [%02x][%02x][%d]\r\n", SPI_in[0], SPI_in[1], config_variables.binBoundriesADC[0]);
#endif

    ps = 0;
    for(i=0 ; i<68 ; i+=4) {
        pFloat = (float *)&SPI_in[i+QPos];
        config_variables.binBoundriesDiametor[ps++] = *pFloat;
    }
#if _DEBUG_MSG_
    printf("binBoundriesDiametor : [%02x][%02x][%02x][%02x][%d]\r\n", SPI_in[QPos], SPI_in[QPos+1], SPI_in[QPos+2], SPI_in[QPos+3],
           (uint16_t)config_variables.binBoundriesDiametor[0]);
#endif
    QPos += 68;

    ps = 0;
    for(i=0 ; i<64 ; i+=4) {
        pFloat = (float *)&SPI_in[i+QPos];
        config_variables.binWeightings[ps++] = *pFloat;
    }
    QPos += 64;

    pFloat = (float *)&SPI_in[QPos];
    config_variables.GSC =  *pFloat;
    QPos += 4;

    pFloat = (float *)&SPI_in[QPos];
    config_variables.SFR =  *pFloat;
    QPos += 4;

    config_variables.ToF_SFR_Factor =   SPI_in[QPos++];

    pFloat = (float *)&SPI_in[QPos];
    config_variables.M_A =  *pFloat;
    QPos += 4;

    pFloat = (float *)&SPI_in[QPos];
    config_variables.M_B =  *pFloat;
    QPos += 4;

    pFloat = (float *)&SPI_in[QPos];
    config_variables.M_C =  *pFloat;
    QPos += 4;

    config_variables.PVP =   SPI_in[QPos++];
    config_variables.power_status =   SPI_in[QPos++];

    pWord = (uint16_t *)&SPI_in[QPos];
    config_variables.max_ToF = *pWord;
    QPos += 2;

    config_variables.laser_ADC =   SPI_in[QPos++];
    config_variables.binWeightingIndex =   SPI_in[QPos++];

    printf("read_Configuration : %d\r\n", QPos);


    return true;
}

#if 0
/***************************************************************************************************************************
  * read Histogram data...max 64...data read
  **************************************************************************************************************************/
bool read_Histogram_data(void)
{
  uint8_t i;
  uint8_t ps;
  uint8_t QPos;
  uint16_t checkSum;

  uint16_t *pWord;
  float       *pFloat;

  if(spidrv_opc_command_ready(OPC_CMD_READ_HISTOGRAM)==false)
      return false;


  memset(SPI_in, 0, sizeof(SPI_in));
  for (i = 0 ; i < 64 ;  i++)  {
      delay_us(10);
      SPI_in[i] = spidrv_transfer(0x01);
  }

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//  BELOW Histogram Data
    QPos = ps = 0;
    for(i=0 ; i<32 ; i+=2) {
        pWord = (uint16_t *)&SPI_in[i];
        Histogram_data.binCount[ps++] = *pWord;
    }
    QPos += 32;

    printf("binCount :  ");
    for(i=0 ; i<16 ; i++) {
        printf("%4d, ", Histogram_data.binCount[i]);
    }
    printf("\r\n");

    Histogram_data.binMToF[0] =   SPI_in[QPos++];       // Bin1 MToF
    Histogram_data.binMToF[1] =   SPI_in[QPos++];       // Bin3 MToF
    Histogram_data.binMToF[2] =   SPI_in[QPos++];       // Bin5 MToF
    Histogram_data.binMToF[3] =   SPI_in[QPos++];       // Bin7 MToF

    printf("%d |%d | %d | %d \r\n", Histogram_data.binMToF[0], Histogram_data.binMToF[1], Histogram_data.binMToF[2], Histogram_data.binMToF[3]);

    pFloat = (float *)&SPI_in[QPos];
    Histogram_data.sampleFlowRate =  *pFloat;

    printf("sampleFlowRate :  [%02x] [%02x] [%02x] [%02x] "LOG_FLOAT_MARKER") \r\n",
               SPI_in[QPos], SPI_in[QPos+1], SPI_in[QPos+2], SPI_in[QPos+3] , LOG_FLOAT(Histogram_data.sampleFlowRate));

    QPos += 4;


    pWord = (uint16_t *)&SPI_in[QPos];
    Histogram_data.Temperature = make_Temperature(*pWord);
    QPos += 2;

    printf("Temperature :  ("LOG_FLOAT_MARKER") : %d\r\n",   LOG_FLOAT(Histogram_data.Temperature), *pWord );


    pWord = (uint16_t *)&SPI_in[QPos];
    Histogram_data.Humidity = make_RelativeHumidity(*pWord);
    QPos += 2;

    printf("Humidity :  ("LOG_FLOAT_MARKER") : %d\r\n",   LOG_FLOAT(Histogram_data.Humidity), *pWord );

    pFloat = (float *)&SPI_in[QPos];
    Histogram_data.samplePeriod =  *pFloat;
    QPos += 4;

    printf("samplePeriod :  "LOG_FLOAT_MARKER" \r\n",   LOG_FLOAT(Histogram_data.samplePeriod) );

    Histogram_data.reject_count_Glitch=   SPI_in[QPos++];
    Histogram_data.reject_count_Long =   SPI_in[QPos++];

    printf("reject_count_Glitch : %d reject_count_Long : %d\r\n",
                     Histogram_data.reject_count_Glitch, Histogram_data.reject_count_Long);

    printf("PM BUFF : ");
    for(i=QPos;i<(QPos+12);i++) {
        printf("%02X, ", SPI_in[i]);
    }
    printf("\r\n");

    pFloat = (float *)&SPI_in[QPos];
    Histogram_data.PM_A_01p0 =  *pFloat;
    QPos += 4;

    pFloat = (float *)&SPI_in[QPos];
    Histogram_data.PM_B_02p5 =  *pFloat;
    QPos += 4;

    pFloat = (float *)&SPI_in[QPos];
    Histogram_data.PM_C_10p0 =  *pFloat;
    QPos += 4;

    printf("PM_A_01p0 :  ("LOG_FLOAT_MARKER") \r\n",   LOG_FLOAT(Histogram_data.PM_A_01p0) );
    printf("PM_B_02p5 :  ("LOG_FLOAT_MARKER") \r\n",   LOG_FLOAT(Histogram_data.PM_B_02p5) );
    printf("PM_C_10p0 :  ("LOG_FLOAT_MARKER") \r\n",   LOG_FLOAT(Histogram_data.PM_C_10p0) );

    pWord = (uint16_t *)&SPI_in[QPos];
    Histogram_data.checkSum = *pWord;

    checkSum = CalcCRC(SPI_in, 62);

    printf("checksun : [%02X] [%02X][%d]\r\n", checkSum, Histogram_data.checkSum, QPos);

    if(Histogram_data.checkSum!=checkSum)
        return false;

    return true;
}
#else
/***************************************************************************************************************************
  *   read PM data...14byte
  **************************************************************************************************************************/
bool read_Histogram_data(void)
{
    uint8_t i;
    uint8_t sens_flag;
    uint8_t OPC_R2_idx;
    uint16_t checkSum;
    uint16_t QPos;

    uint16_t *pWord;
   float *pFloat;

#if 0
   float SFR;
   float Period;
   float ml_per_period;
#endif

    QPos = 0;
    sens_flag = GET_OPC_R2_ALL_STATE();
    for(OPC_R2_idx=OPC_R2_NUM_1;OPC_R2_idx<=OPC_R2_NUM_8;OPC_R2_idx++) {
        if((sens_flag&(1<<i))==(1<<i)) {

            set_OPC_R2_latch_address(OPC_R2_idx);
            if(spidrv_opc_command_ready(OPC_CMD_READ_HISTOGRAM)==true) {

                memset(SPI_in, 0, sizeof(SPI_in));
                for (i = 0 ; i < 64 ;  i++)  {
                    delay_us(10);
                    SPI_in[i] = spidrv_transfer(0x00);

//                    printf(" [%02X]", SPI_in[i]);

                }
//                printf("\r\n");

                pWord = (uint16_t *)&SPI_in[62];
                checkSum = CalcCRC(SPI_in, 62);

#if _DEBUG_MSG_
                printf("checksun : [%d] [%02X] [%02X]\r\n", OPC_R2_idx, checkSum, *pWord);
#endif

                if(*pWord==checkSum) {
                      memcpy(&OPC_R2_buff[QPos], SPI_in, 64);
                }
            }
        }

        QPos+=64;
    }

    return true;
}
#endif


/***************************************************************************************************************************
  * read Histogram data...max 64...data read
  **************************************************************************************************************************/
bool read_Configuration_Variable(void)
{
  uint8_t i;
  uint8_t QPos;

  uint16_t *pWord;
  float       *pFloat;

  float pFloat_Buff[20];
  float ml_per_period;

  if(spidrv_opc_command_ready(OPC_CMD_READ_CONFIG)==false)
      return false;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//  Bin Boundaries ADC
    memset(SPI_in, 0, sizeof(SPI_in));
    for (i = 0 ; i < 193 ;  i++)  {
        delay_us(10);
        SPI_in[i] = spidrv_transfer(0x01);
    }


#if _DEBUG_MSG_
    printf("Bin Boundaries ADC : ");
#endif

    QPos = 0;
    for(i=0 ; i<17 ; i++) {
        pWord = (uint16_t *)&SPI_in[QPos];
        QPos+=2;

#if _DEBUG_MSG_
        printf("%d, ", *pWord);
#endif
    }

#if _DEBUG_MSG_
    printf("[%d]\r\n", QPos);
#endif
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//  Bin Boundaries diameter
#if _DEBUG_MSG_
    printf("Bin Boundaries diameter : ");
#endif
    for(i=0 ; i<17 ; i++) {
        pFloat = (float *)&SPI_in[QPos];
        pFloat_Buff[i] = *pFloat;
        QPos+=4;
    }
#if _DEBUG_MSG_
    for(i=0 ; i<17 ; i++) {
        printf("("LOG_FLOAT_MARKER") , ",  LOG_FLOAT(pFloat_Buff[i]));
    }
    printf("\r\n");

    printf("Bin Weightings : %d", QPos);
#endif

    for(i=0 ; i<16 ; i++) {
        pFloat = (float *)&SPI_in[QPos];
        pFloat_Buff[i] = *pFloat;
        QPos+=4;
    }

#if _DEBUG_MSG_
    for(i=0 ; i<16 ; i++) {
        printf("("LOG_FLOAT_MARKER") , ",  LOG_FLOAT(pFloat_Buff[i]));
    }
    printf("\r\n");

    printf("GSC : ");
#endif
    pFloat = (float *)&SPI_in[QPos];
    ml_per_period = *pFloat;
#if _DEBUG_MSG_
    printf("  ("LOG_FLOAT_MARKER") \r\n",  LOG_FLOAT(ml_per_period));
#endif

    QPos += 4;
#if _DEBUG_MSG_
    printf("Sample Flow Rate : ");
#endif
    pFloat = (float *)&SPI_in[QPos];
    ml_per_period = *pFloat;
#if _DEBUG_MSG_
    printf("  ("LOG_FLOAT_MARKER") \r\n ",  LOG_FLOAT(ml_per_period));
#endif
    QPos += 4;

#if _DEBUG_MSG_
    printf("SFR Factor :    : %d\r\n ",  SPI_in[QPos]);
#endif
    QPos++;

    pFloat = (float *)&SPI_in[QPos];
    ml_per_period = *pFloat;
#if _DEBUG_MSG_
    printf("M_A :   ("LOG_FLOAT_MARKER") \r\n ",  LOG_FLOAT(ml_per_period));
#endif

    QPos += 4;
    pFloat = (float *)&SPI_in[QPos];
    ml_per_period = *pFloat;
#if _DEBUG_MSG_
    printf("M_B :   ("LOG_FLOAT_MARKER") \r\n ",  LOG_FLOAT(ml_per_period));
#endif

    QPos += 4;
    pFloat = (float *)&SPI_in[QPos];
    ml_per_period = *pFloat;
#if _DEBUG_MSG_
    printf("M_C :   ("LOG_FLOAT_MARKER") \r\n ",  LOG_FLOAT(ml_per_period));
#endif

    QPos += 4;
    printf("PVP : %d \r\n",  SPI_in[QPos++]);
    printf("Power State : %02X\r\n ",  SPI_in[QPos++]);

    pWord = (uint16_t *)&SPI_in[i];
    printf("Max TOF : %d\r\n ", *pWord);

    QPos += 2;
    printf("Laser DAC : %d \r\n",  SPI_in[QPos++]);
    printf("BinWeightingIndex : %02X\r\n",  SPI_in[QPos++]);
}

/***************************************************************************************************************************
  * write OPC power control
  **************************************************************************************************************************/
bool set_Peripherial_Power_Control(uint8_t power_mode)
 {

    if(spidrv_opc_command_ready(OPC_CMD_WRITE_POWER_STATE)==false)
        return false;

    if(spidrv_opc_command_ready(power_mode)==false)
        return false;

    return true;
 }

bool Digital_pot_Set_Laser_Power(uint8_t LaserDAC)
 {
    if(spidrv_opc_command_ready(OPC_CMD_WRITE_LASER_POWER)==false)
        return false;

    if(spidrv_opc_command_ready(LaserDAC)==false)
        return false;

    return true;
 }

void OPC_R2_Device_init(void)
{
    uint8_t i;
    uint8_t sens_flag=0;

    for(i=OPC_R2_NUM_1;i<=OPC_R2_NUM_8;i++) {
        set_OPC_R2_latch_address(i);

        if(read_Information()==true) {
            sens_flag|=(1<<i);
        }
    }

    SET_OPC_R2_ALL_STATE(sens_flag);

    OPC_R2_info.ini_count++;
    if((sens_flag==0xFF) ||(OPC_R2_info.ini_count>=3)) {          //
        OPC_R2_info.ini_count = 0;
        OPC_R2_info.device_step = OPC_R2_FAN_POWER_ON;
    }
}

void OPC_R2_Fan_Power_On(void)
{
    uint8_t i;
    uint8_t sens_flag=0;
    uint8_t sens_state=0;

    sens_flag = GET_OPC_R2_ALL_STATE();

    for(i=OPC_R2_NUM_1;i<=OPC_R2_NUM_8;i++) {
          if((sens_flag&(1<<i))==(1<<i)) {
              set_OPC_R2_latch_address(i);

              if(set_Peripherial_Power_Control(FAN_LASER_POWER_ON)==true) {
                  sens_state|=(1<<i);
              }
          }
    }

    OPC_R2_info.ini_count++;
    if((sens_flag==sens_state) ||(OPC_R2_info.ini_count>=3)) {    // 3회 이상 ...
        OPC_R2_info.device_step = OPC_R2_LASER_POWER;
        OPC_R2_info.ini_count = 0;
        OPC_R2_info.wait_time= 0;
    }
}

void OPC_R2_Digital_Laser_Power(void)
{
    uint8_t i;
    uint8_t sens_flag=0;

    sens_flag = GET_OPC_R2_ALL_STATE();

    for(i=OPC_R2_NUM_1;i<=OPC_R2_NUM_8;i++) {
          if((sens_flag&(1<<i))==(1<<i)) {
              set_OPC_R2_latch_address(i);

              if(Digital_pot_Set_Laser_Power(242)==true) {

              }
          }
    }

    OPC_R2_info.device_step = OPC_R2_FAN_POWER_ON_WAIT;
    OPC_R2_info.ini_count = 0;
    OPC_R2_info.wait_time= 0;

}

void OPC_R2_Histogram_data(void )
{
  uint8_t i;
  uint8_t sens_flag=0;
  uint8_t sens_state=0;

  sens_flag = GET_OPC_R2_ALL_STATE();

  for(i=OPC_R2_NUM_1;i<=OPC_R2_NUM_8;i++) {
        if((sens_flag&(1<<i))==(1<<i)) {
            set_OPC_R2_latch_address(i);

            if(set_Peripherial_Power_Control(FAN_LASER_POWER_ON)==true) {
                sens_state|=(1<<i);
            }
        }
  }
}


uint8_t OPC_R2_Connection(uint8_t ps)
{
    uint8_t sens_flag=0;
    sens_flag = GET_OPC_R2_ALL_STATE();

    if((sens_flag&(1<<ps))==(1<<ps)) {
        return 1;
    }
    else {
        return 0;
    }


}
/**************************************************************************************************************************
  *
  *   OPC-R2 단계별 동작 상황...1sec Interval Time Callback
  *
  **************************************************************************************************************************/
static void spidrv_transfer_handler_callback(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;

        switch(OPC_R2_info.device_step) {
            case OPC_R2_READY:
                break;

            case OPC_R2_DEVICE_INIT:
                OPC_R2_Device_init();
                break;

            case OPC_R2_FAN_POWER_ON:
                OPC_R2_Fan_Power_On();
                break;

            case OPC_R2_LASER_POWER:
              OPC_R2_Digital_Laser_Power();
              break;

            case OPC_R2_FAN_POWER_ON_WAIT:                                    // FAN & LASER POWER ON READY...
                OPC_R2_info.wait_time++;
                if(OPC_R2_info.wait_time >= 5) {

                    read_Configuration_Variable();                                        // CONFIG VALUE

                    OPC_R2_info.device_step = OPC_R2_HISTOGRAM;
                    memset(OPC_R2_buff, 0, sizeof(OPC_R2_buff));      // BUFFER CLEAR

                }
                break;

            case OPC_R2_HISTOGRAM:
              OPC_R2_info.vbat_lvl_mv = read_batt_lvl_mV();
                if(read_Histogram_data()==true) {         // DATA READ OK...SEND to APK
                    sl_bt_external_signal(EXT_SENS_DATA_SEND);
                }
                break;

/*****************************************************************************************************************
  *   SIMULATION PROGRAM
  *****************************************************************************************************************/
            case OPC_R2_SIMULATION:
                {
                    uint8_t i;
                    uint8_t OPC_R2_idx;
                    uint16_t QPos=0;

                    OPC_R2_info.vbat_lvl_mv = read_batt_lvl_mV();
                    for(OPC_R2_idx=OPC_R2_NUM_1;OPC_R2_idx<=OPC_R2_NUM_8;OPC_R2_idx++) {

                        for(i=0;i<64;i++) {
                            SPI_in[i] = Histrogram_tbl_buff[Histrogram_sample_pos][i];
                        }

                        memcpy(&OPC_R2_buff[QPos], SPI_in, 64);
                        QPos+=64;

                    }

                    sl_bt_external_signal(EXT_SENS_DATA_SEND);

                    Histrogram_sample_pos++;
                    Histrogram_sample_pos %=20;

//                    printf("Histrogram_sample_pos : %d \r\n",  Histrogram_sample_pos);

                }

              break;

            default:
                break;
        }
}




/**********************************************************************************************
 * Set up periodic  measurement timer. 5sec
 ***********************************************************************************************/
void run_spidrv_transfer_timer(void)
{
    sl_sleeptimer_start_periodic_timer_ms(&spidrv_transfer_timer,
                                          SPIDRV_TRANSFER_INTERVAL_MS,
                                          spidrv_transfer_handler_callback,
                                          NULL, 0, 0);
}
