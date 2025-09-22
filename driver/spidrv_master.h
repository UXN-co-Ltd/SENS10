/***************************************************************************//**
 * @file
 * @brief spidrv baremetal examples functions
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

#ifndef SPIDRV_MASTER_H
#define SPIDRV_MASTER_H


#define LOG_FLOAT_MARKER    "%c%ld.%04ld"
#define LOG_FLOAT_MARKER1 "%c%ld.%02ld"

/**
 * @brief Macro for dissecting a float number into two numbers (integer and residuum).
 */
#define LOG_FLOAT(val) (val > 0 || val <=-1) ?' ':'-',        \
                      (int32_t)(val),                                                         \
                       (int32_t)(((val > 0) ? (val) - (int32_t)(val)       \
                       : (int32_t)(val) - (val))*10000)


/*****************************************************************************
 * Define
 ******************************************************************************/
#define OPC_TRIES_TMOUT          0xFF
#define OPC_MAX_TRIES           10


#define OPC_READY_OK          1
#define OPC_READY_FAIL        0

#define OPC_READY             0xF3
#define OPC_BUSY                0x31

#define OPC_CMD_READY             0xF3
#define OPC_CMD_BUSY                0x31

//# Most opcodes are shared among different device versions
#define OPC_CMD_WRITE_POWER_STATE     0x03
#define OPC_CMD_WRITE_LASER_POWER     0x04
#define OPC_CMD_READ_POWER_STATE        0x13
#define OPC_CMD_READ_INFO_STRING           0x3F
#define OPC_CMD_READ_SERIAL_STRING      0x10
#define OPC_CMD_READ_FW_VERSION           0x12
#define OPC_CMD_READ_HISTOGRAM             0x30
#define OPC_CMD_READ_PM                               0x32
#define OPC_CMD_READ_CONFIG                       0x3C
#define OPC_CMD_WRITE_CONFIG                    0x3A
#define OPC_CMD_CHECK_STATUS                    0xCF
#define OPC_CMD_RESET                                     0x06
//# OPC-N3 peripheral power status "OptionByte" flags. See 072-0503.
#define OPC_N3_POPT_FAN_POT                         1
#define OPC_N3_POPT_LASER_POT                   2
#define OPC_N3_POPT_LASER_SWITCH            3
#define OPC_N3_POPT_GAIN_TOGGLE               4

#define FAN_LASER_POWER_ON                0x03
#define FAN_LASER_POWER_OFF              0x00


typedef enum {
    OPC_R2_READY=0,
    OPC_R2_DEVICE_INIT,
    OPC_R2_FAN_POWER_ON,
    OPC_R2_LASER_POWER,
    OPC_R2_FAN_POWER_ON_WAIT,
    OPC_R2_HISTOGRAM,
    OPC_R2_SIMULATION,
    OPC_R2_MAX
}OPC_R2_state_e;


enum {
  OPC_R2_NUM_1=0,
  OPC_R2_NUM_2,
  OPC_R2_NUM_3,
  OPC_R2_NUM_4,
  OPC_R2_NUM_5,
  OPC_R2_NUM_6,
  OPC_R2_NUM_7,
  OPC_R2_NUM_8,
  OPC_R2_NUM_MAX
};

/*****************************************************************************
 * FLAG
 ******************************************************************************/
typedef struct {
  uint8_t bit0:1;
  uint8_t bit1:1;
  uint8_t bit2:1;
  uint8_t bit3:1;
  uint8_t bit4:1;
  uint8_t bit5:1;
  uint8_t bit6:1;
  uint8_t bit7:1;
} tyFLAGBITS;

typedef union {
  uint8_t bAll;
  tyFLAGBITS b;
} tyFLAG;



/*****************************************************************************
 *    struct 변수
 ******************************************************************************/

typedef struct  {
    uint8_t major;
    uint8_t minor;
}firmwareVersion;

typedef struct  {
    uint16_t binBoundriesADC[17];             // BB0 ~ BB16 17 * 2 = 34;
    float binBoundriesDiametor[17];         // BB0 ~ BB16 17 *4 = 68;
    float binWeightings[16];                         // BW0 ~ BW15 16 *4 = 64;
    float GSC;                                                      // GCS = 4  Gain Scaling Coefficient
    float SFR;                                                      // SFR = 4  Sample Flow Rate
    uint8_t ToF_SFR_Factor;                             // 1
    float M_A;                                                      //  4 Particle mass Concentration A
    float M_B;                                                       // 4
    float M_C;                                                       // 4
    uint8_t PVP;                                                    // 1  Particle Validation Period)
    uint8_t power_status;                                    // 1
    uint16_t max_ToF;                                          // 2
    uint8_t laser_ADC;                                         // 1
    uint8_t binWeightingIndex;                          // 1
} config_variables_t;
//configurationVariables;

extern config_variables_t config_variables;


typedef struct  {
    uint16_t binCount[16];             // BB0 ~ BB15 16 * 2 = 32;

    uint8_t   binMToF[4]  ;               // bin1, bun3, bin5, bin7

    float  sampleFlowRate;             // sample Flow Rate Byte0 ~ samplePeriod Byte3
    float  samplePeriod;                 //  samplePeriod Byte0 ~ samplePeriod Byte3

    float Temperature;
    float Humidity ;

    uint8_t reject_count_Glitch;
    uint8_t reject_count_Long;

    float PM_A_01p0     ;
    float PM_B_02p5   ;
    float PM_C_10p0   ;

    uint16_t checkSum;

}Histogram_data_t;

extern Histogram_data_t Histogram_data;

typedef struct {
    float PM_A_01p0 ;
    float PM_B_02p5;
    float PM_C_10p0;

    uint16_t checkSum;

}PM_data_t;

extern PM_data_t PM_data;


typedef struct  {
  OPC_R2_state_e  device_step;
  tyFLAG  sens_state;

  uint8_t latch_pos;
  uint8_t ini_count;
  uint8_t ini_step;
  uint8_t wait_time;

  uint8_t mac_address[4];
  float vbat_lvl_mv;

} OPC_R2_info_t;

extern OPC_R2_info_t OPC_R2_info;

#define MAX_OPC_R2_SIZE   (512+8)
extern unsigned char OPC_R2_buff[MAX_OPC_R2_SIZE];

/*****************************************************************************
 * Initialize example
 ******************************************************************************/

void run_spidrv_transfer_timer(void);

/*****************************************************************************
 * Ticking function
 ******************************************************************************/

int spidrv_read_reg_action(uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
int spidrv_write_reg_action(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);

int spidrv_read_action( uint8_t reg, uint32_t addr,  uint8_t * rbuffer, uint32_t rlen);
int spidrv_write_action( uint8_t reg, uint32_t addr,  const uint8_t * wbuffer, uint32_t wlen);

void spidrv_app_init(void);

int spidrv_write_opc_reg_action(const  uint8_t reg,  const uint8_t  val, const uint8_t  wlen);
int spidrv_read_opc_reg_action( uint8_t reg, uint8_t * rbuffer, uint32_t rlen);

bool  set_OPC_R2_reset(void);

uint8_t OPC_R2_Connection(uint8_t ps);

#endif  // SPIDRV_MASTER_H
