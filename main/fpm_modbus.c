#include "stdbool.h"
#include "string.h"
#include "cJSON.h"
#include "esp_err.h"
#include "inttypes.h"
#include "esp_log.h"
#include "stddef.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "stdio.h"
#include "total_app.h"

#define MODBUS_UART_NUM UART_NUM_1
#define HOLD_OFFSET_RW(field) ((uint16_t)(offsetof(holding_reg_rw_params_t, field)))
#define MODBUS_SERIAL_RX_BUFFER_SIZE 20
#define MODBUS_GET_TIMEOUT 150
#define UARTINIT_DELAY 10

//4ms
#define T3_5 0
//1.8ms
#define T1_5 1

#define BIT31   0x80000000
#define BIT30   0x40000000
#define BIT29   0x20000000
#define BIT28   0x10000000
#define BIT27   0x08000000
#define BIT26   0x04000000
#define BIT25   0x02000000
#define BIT24   0x01000000
#define BIT23   0x00800000
#define BIT22   0x00400000
#define BIT21   0x00200000
#define BIT20   0x00100000
#define BIT19   0x00080000
#define BIT18   0x00040000
#define BIT17   0x00020000
#define BIT16   0x00010000
#define BIT15   0x00008000
#define BIT14   0x00004000
#define BIT13   0x00002000
#define BIT12   0x00001000
#define BIT11   0x00000800
#define BIT10   0x00000400
#define BIT9     0x00000200
#define BIT8     0x00000100
#define BIT7     0x00000080
#define BIT6     0x00000040
#define BIT5     0x00000020
#define BIT4     0x00000010
#define BIT3     0x00000008
#define BIT2     0x00000004
#define BIT1     0x00000002
#define BIT0     0x00000001

#define STR(fieldname) ((const char *)(fieldname))

typedef enum {MODBUS_GETADDY, MODBUS_GETFUNC , MODBUS_GETLEN, MODBUS_GET_REGISTER_ADDRESS, MODBUS_GETDATA, MODBUS_GETQUANTITY, MODBUS_GETCRC, MODBUS_RXCOMPLETE, MODBUS_WAIT_TIMEOUT}_modbus_serial_state;

typedef enum {ILLEGAL_FUNCTION=1,ILLEGAL_DATA_ADDRESS=2,
ILLEGAL_DATA_VALUE=3,SLAVE_DEVICE_FAILURE=4,ACKNOWLEDGE=5,SLAVE_DEVICE_BUSY=6,
MEMORY_PARITY_ERROR=8,GATEWAY_PATH_UNAVAILABLE=10,GATEWAY_TARGET_NO_RESPONSE=11,
SERIAL_MALF=1, CRC_MISM=2, NO_RESP=3} exception;

typedef enum _function{FUNC_READ_COILS=0x01,FUNC_READ_DISCRETE_INPUT=0x02,
FUNC_READ_HOLDING_REGISTERS=0x03,FUNC_READ_INPUT_REGISTERS=0x04,
FUNC_WRITE_SINGLE_COIL=0x05,FUNC_WRITE_SINGLE_REGISTER=0x06,
FUNC_READ_EXCEPTION_STATUS=0x07,FUNC_DIAGNOSTICS=0x08,
FUNC_GET_COMM_EVENT_COUNTER=0x0B,FUNC_GET_COMM_EVENT_LOG=0x0C,
FUNC_WRITE_MULTIPLE_COILS=0x0F,FUNC_WRITE_MULTIPLE_REGISTERS=0x10,
FUNC_REPORT_SLAVE_ID=0x11,FUNC_READ_FILE_RECORD=0x14,
FUNC_WRITE_FILE_RECORD=0x15,FUNC_MASK_WRITE_REGISTER=0x16,
FUNC_READ_WRITE_MULTIPLE_REGISTERS=0x17,FUNC_READ_FIFO_QUEUE=0x18} function;

typedef struct
{
   uint8_t address;
   function func;
   uint8_t len;
   uint8_t add[2];
   exception error_code[2];
   uint8_t qty[2];
   uint8_t data[MODBUS_SERIAL_RX_BUFFER_SIZE];
   uint8_t crc[2];
}modbus_dt_struct;

union
{
   uint8_t b[2];
   uint16_t d;
}modbus_serial_crc;

typedef enum {
    MB_PARAM_HOLDING = 0x00,         /*!< Modbus Holding register. */
    MB_PARAM_INPUT,                  /*!< Modbus Input register. */
    MB_PARAM_COIL,                   /*!< Modbus Coils. */
    MB_PARAM_DISCRETE,               /*!< Modbus Discrete bits. */
    MB_PARAM_COUNT,
    MB_PARAM_UNKNOWN = 0xFF
} mb_param_type_t;

typedef enum {
    PARAM_TYPE_U16 = 0x01,                  /*!< Unsigned 16 */
    PARAM_TYPE_U32 = 0x02,                  /*!< Unsigned 32 */
    PARAM_TYPE_FLOAT = 0x03,                /*!< Float type */
    PARAM_TYPE_ASCII = 0x04,                /*!< ASCII type */
    PARAM_TYPE_BIN16 = 0x07,                /*!< BIN 16 type */
    PARAM_TYPE_BIN32 = 0x08,                /*!< BIN 32 type */
    PARAM_TYPE_HEX16 = 0x09,                /*!< HEX 16 type */
    PARAM_TYPE_HEX32 = 0x0A,                /*!< HEX 32 type */
} mb_descr_type_t;

typedef enum {
    PAR_PERMS_READ               = 1 << BIT0,                                   /**< the characteristic of the device are readable */
    PAR_PERMS_WRITE              = 1 << BIT1,                                   /**< the characteristic of the device are writable*/
    PAR_PERMS_WRITE_MULTIPLE     = 1 << BIT2,                                   /**< the characteristic of the device are writable*/
} mb_param_perms_t;

typedef enum {
    PARAM_SIZE_U8 = 0x01,                   /*!< Unsigned 8 */
    PARAM_SIZE_U8_REG = 0x02,               /*!< Unsigned 8, register value */
    PARAM_SIZE_I8_REG = 0x02,               /*!< Signed 8, register value */
    PARAM_SIZE_I16 = 0x02,                  /*!< Unsigned 16 */
    PARAM_SIZE_U16 = 0x02,                  /*!< Unsigned 16 */
    PARAM_SIZE_I32 = 0x04,                  /*!< Signed 32 */
    PARAM_SIZE_U32 = 0x04,                  /*!< Unsigned 32 */
    PARAM_SIZE_FLOAT = 0x04,                /*!< Float 32 size */
    PARAM_SIZE_ASCII = 0x08,                /*!< ASCII size default*/
    PARAM_SIZE_ASCII24 = 0x18,              /*!< ASCII24 size */
    PARAM_SIZE_I64 = 0x08,                  /*!< Signed integer 64 size */
    PARAM_SIZE_U64 = 0x08,                  /*!< Unsigned integer 64 size */
    PARAM_SIZE_DOUBLE = 0x08,               /*!< Double 64 size */
    PARAM_MAX_SIZE
} mb_descr_size_t;

typedef struct
{
    bool                *modbus_operation_enable;
    uint8_t             *modbus_try_cnt;
    uint16_t            cid;                /*!< Characteristic cid */
    const char*         param_key;          /*!< The key (name) of the parameter */
    const char*         param_units;        /*!< The physical units of the parameter */
    uint8_t             mb_slave_addr;      /*!< Slave address of device in the Modbus segment */
    mb_param_type_t     mb_param_type;      /*!< Type of modbus parameter */
    uint16_t            mb_reg_start;       /*!< This is the Modbus register address. This is the 0 based value. */
    uint16_t            mb_size;            /*!< Size of mb parameter in registers */
    uint16_t            param_offset;       /*!< Parameter name (OFFSET in the parameter structure) */
    mb_descr_type_t     param_type;         /*!< Float, U8, U16, U32, ASCII, etc. */
    mb_descr_size_t     param_size;         /*!< Number of bytes in the parameter. */
    mb_param_perms_t    access;             /*!< Access permissions based on mode */
    bool                *modbus_operation_result;
    exception           *error_code;
} modbus_operation_parameter_descriptor_t;

enum
{
        MB_DEVICE_ADDR1 = 1
};

enum
{
    CID_R_4000_Serial_number_2__HEX,
    CID_R_4002_Meter_code_1__HEX,
    CID_R_4003_Modbus_ID_1__Signed,
    CID_R_4004_Baud_rate_1__Signed,
    CID_R_4005_Protocol_version_2__Float,
    CID_R_4007_Software_version_2__Float,
    CID_R_4009_Hardware_version_2__Float,
    CID_R_400B_Meter_amps_1_A_Signed,
    CID_R_400C_CT_ratio_1_A_HEX,
    CID_R_400D_S0_output_rate_2_kWh_Float,
    CID_R_400F_Combination_code_1__HEX,
    CID_R_4010_LCD_cycle_time_1_sec_HEX,
    CID_R_4011_Parity_setting_1__Signed,
    CID_R_4012_Current_direction_1__ASCII,
    CID_R_4013_L2_Current_direction_1__ASCII,
    CID_R_4014_L3_Current_direction_1__ASCII,
    CID_R_4016_Power_down_counter_1__Signed,
    CID_R_4017_Present_quadrant_1__Signed,
    CID_R_4018_L1_Quadrant_1__Signed,
    CID_R_4019_L2_Quadrant_1__Signed,
    CID_R_401A_L3_Quadrant_1__Signed,
    CID_R_401B_Checksum_2__HEX,
    CID_R_401D_Active_status_word_2__HEX,
    CID_R_401F_CT_ratio_2_A_Signed,
    CID_R_4021_Pulse_width_2_ms_Signed,
    CID_R_4022_Pulse_type_setting_1_HEX,
    CID_R_4023_Checksum_2_2__HEX,
    CID_R_4026_Data_type_setting_1__Signed,
    CID_R_4032_Screen_direction_1__Signed,
    CID_R_4033_OBIS_code_1__Signed,
    CID_R_5000_Voltage_2_V_Float,
    CID_R_5002_L1_Voltage_2_V_Float,
    CID_R_5004_L2_Voltage_2_V_Float,
    CID_R_5006_L3_Voltage_2_V_Float,
    CID_R_5008_Grid_frequency_2_Hz_Float,
    CID_R_500A_Current_2_A_Float,
    CID_R_500C_L1_Current_2_A_Float,
    CID_R_500E_L2_Current_2_A_Float,
    CID_R_5010_L3_Current_2_A_Float,
    CID_R_5012_Total_active_power_2_kW_Float,
    CID_R_5014_L1_Active_power_2_kW_Float,
    CID_R_5016_L2_Active_power_2_kW_Float,
    CID_R_5018_L3_Active_power_2_kW_Float,
    CID_R_501A_Total_reactive_power_2_kvar_Float,
    CID_R_501C_L1_Reactive_power_2_kvar_Float,
    CID_R_501E_L2_Reactive_power_2_kvar_Float,
    CID_R_5020_L3_Reactive_power_2_kvar_Float,
    CID_R_5022_Total_apparent_power_2_kVA_Float,
    CID_R_5024_L1_Apparent_power_2_kVA_Float,
    CID_R_5026_L2_Apparent_power_2_kVA_Float,
    CID_R_5028_L3_Apparent_power_2_kVA_Float,
    CID_R_502A_Power_factor_2_Float,
    CID_R_502C_L1_Power_factor_2_Float,
    CID_R_502E_L2_Power_factor_2_Float,
    CID_R_5030_L3_Power_factor_2_Float,
    CID_R_5032_L1_L2_Voltage_2_V_Float,
    CID_R_5034_L1_L3_Voltage_2_V_Float,
    CID_R_5036_L2_L3_Voltage_2_V_Float,
    CID_R_6000_Total_active_energy_2_kWh_Float,
    CID_R_6002_T1_Total_active_energy_2_kWh_Float,
    CID_R_6004_T2_Total_active_energy_2_kWh_Float,
    CID_R_6006_L1_Total_active_energy_2_kWh_Float,
    CID_R_6008_L2_Total_active_energy_2_kWh_Float,
    CID_R_600A_L3_Total_active_energy_2_kWh_Float,
    CID_R_600C_Forward_active_energy_2_kWh_Float,
    CID_R_600E_T1_Forward_active_energy_2_kWh_Float,
    CID_R_6010_T2_Forward_active_energy_2_kWh_Float,
    CID_R_6012_L1_Forward_active_energy_2_kWh_Float,
    CID_R_6014_L2_Forward_active_energy_2_kWh_Float,
    CID_R_6016_L3_Forward_active_energy_2_kWh_Float,
    CID_R_6018_Reverse_active_energy_2_kWh_Float,
    CID_R_601A_T1_Reverse_active_energy_2_kWh_Float,
    CID_R_601C_T2_Reverse_active_energy_2_kWh_Float,
    CID_R_601E_L1_Reverse_active_energy_2_kWh_Float,
    CID_R_6020_L2_Reverse_active_energy_2_kWh_Float,
    CID_R_6022_L3_Reverse_active_energy_2_kWh_Float,
    CID_R_6024_Total_reactive_energy_2_kvarh_Float,
    CID_R_6026_T1_Total_reactive_energy_2_kvarh_Float,
    CID_R_6028_T2_Total_reactive_energy_2_kvarh_Float,
    CID_R_602A_L1_Total_reactive_energy_03_kvarh_Float,
    CID_R_602C_L2_Total_reactive_energy_2_kvarh_Float,
    CID_R_602E_L3_Total_reactive_energy_2_kvarh_Float,
    CID_R_6030_Forward_reactive_energy_2_kvarh_Float,
    CID_R_6032_T1_Forward_reactive_energy_2_kvarh_Float,
    CID_R_6034_T2_Forward_reactive_energy_2_kvarh_Float,
    CID_R_6036_L1_Forward_reactive_energy_2_kvarh_Float,
    CID_R_6038_L2_Forward_reactive_energy_2_kvarh_Float,
    CID_R_603A_L3_Forward_reactive_energy_2_kvarh_Float,
    CID_R_603C_Reverse_reactive_energy_2_kvarh_Float,
    CID_R_603E_T1_Reverse_reactive_energy_2_kvarh_Float,
    CID_R_6040_T2_Reverse_reactive_energy_2_kvarh_Float,
    CID_R_6042_L1_Reverse_reactive_energy_2_kvarh_Float,
    CID_R_6044_L2_Reverse_reactive_energy_2_kvarh_Float,
    CID_R_6046_L3_Reverse_reactive_energy_2_kvarh_Float,
    CID_R_6048_Tariff_1__Signed,
    CID_R_6049_Resettable_day_register_2_kWh_Float,
    CID_R_604B_T3_Total_active_energy_2_kWh_Float,
    CID_R_604D_T4_Total_active_energy_2_kWh_Float,
    CID_R_604F_T3_Forward_active_energy_2_kWh_Float,
    CID_R_6051_T4_Forward_active_energy_2_kWh_Float,
    CID_R_6053_T3_Reverse_active_energy_2_kWh_Float,
    CID_R_6055_T4_Reverse_active_energy_2_kWh_Float,
    CID_R_6057_T3_Total_reactive_energy_2_kvarh_Float,
    CID_R_6059_T4_Total_reactive_energy_2_kvarh_Float,
    CID_R_605B_T3_Forward_reactive_energy_2_kvarh_Float,
    CID_R_605D_T4_Forward_reactive_energy_2_kvarh_Float,
    CID_R_605F_T3_Reverse_reactive_energy_2_kvarh_Float,
    CID_R_6061_T4_Reverse_reactive_energy_2_kvarh_Float,
    CID_R_6063_Imp_Inductive_reactive_energy_in_Q1_total_2_kWh_Float,
    CID_R_6065_Imp_Inductive_reactive_energy_in_Q1_T1_2_kWh_Float,
    CID_R_6067_Imp_Inductive_reactive_energy_in_Q1_T2_2_kWh_Float,
    CID_R_6069_Imp_Inductive_reactive_energy_in_Q1_T3_2_kWh_Float,
    CID_R_606B_Imp_Inductive_reactive_energy_in_Q1_T4_2_kWh_Float,
    CID_R_606D_Imp_capacitive_reactive_energy_in_Q2_total_2_kWh_Float,
    CID_R_606F_Imp_capacitive_reactive_energy_in_Q2_T1_2_kWh_Float,
    CID_R_6071_Imp_capacitive_reactive_energy_in_Q2_T2_2_kWh_Float,
    CID_R_6073_Imp_capacitive_reactive_energy_in_Q2_T3_03_2_kWh_Float,
    CID_R_6075_Imp_capacitive_reactive_energy_in_Q2_T4_03_2_kWh_Float,
    CID_R_6077_Exp_Inductive_reactive_energy_in_Q3_total_2_kWh_Float,
    CID_R_6079_Exp_Inductive_reactive_energy_in_Q3_T1_2_kWh_Float,
    CID_R_607B_Exp_Inductive_reactive_energy_in_Q3_T2_2_kWh_Float,
    CID_R_607D_Exp_Inductive_reactive_energy_in_Q3_T3_2_kWh_Float,
    CID_R_607F_Exp_Inductive_reactive_energy_in_Q3_T4_2_kWh_Float,
    CID_R_6081_Exp_capacitive_reactive_energy_in_Q4_total_2_kWh_Float,
    CID_R_6083_Exp_capacitive_reactive_energy_in_Q4_T1_2_kWh_Float,
    CID_R_6085_Exp_capacitive_reactive_energy_in_Q4_T2_2_kWh_Float,
    CID_R_6087_Exp_capacitive_reactive_energy_in_Q4_T3_2_kWh_Float,
    CID_R_6089_Exp_capacitive_reactive_energy_in_Q4_T4_2_kWh_Float,
    CID_R_608B_Resettable_day_counter_L1_2_kWh_Float,
    CID_R_608D_Resettable_day_counter_L2_2_kWh_Float,
    CID_R_608F_Resettable_day_counter_L3_2_kWh_Float,
    CID_RW_COUNT,
};

typedef enum
{
    MODBUS_ITERATE_CID, 
    MODBUS_READ_WAIT,
    MODBUS_WRITE_GENERIC_WAIT,
    MODBUS_UARTINIT_DELAY
}_enum_internal_modbus_operation;

typedef struct
{
    float CID_R_4000_Serial_number_2__HEX_t;
    float CID_R_4002_Meter_code_1__HEX_t;
    float CID_R_4003_Modbus_ID_1__Signed_t;
    float CID_R_4004_Baud_rate_1__Signed_t;
    float CID_R_4005_Protocol_version_2__Float_t;
    float CID_R_4007_Software_version_2__Float_t;
    float CID_R_4009_Hardware_version_2__Float_t;
    float CID_R_400B_Meter_amps_1_A_Signed_t;
    float CID_R_400C_CT_ratio_1_A_HEX_t;
    float CID_R_400D_S0_output_rate_2_kWh_Float_t;
    float CID_R_400F_Combination_code_1__HEX_t;
    float CID_R_4010_LCD_cycle_time_1_sec_HEX_t;
    float CID_R_4011_Parity_setting_1__Signed_t;
    float CID_R_4012_Current_direction_1__ASCII_t;
    float CID_R_4013_L2_Current_direction_1__ASCII_t;
    float CID_R_4014_L3_Current_direction_1__ASCII_t;
    float CID_R_4016_Power_down_counter_1__Signed_t;
    float CID_R_4017_Present_quadrant_1__Signed_t;
    float CID_R_4018_L1_Quadrant_1__Signed_t;
    float CID_R_4019_L2_Quadrant_1__Signed_t;
    float CID_R_401A_L3_Quadrant_1__Signed_t;
    float CID_R_401B_Checksum_2__HEX_t;
    float CID_R_401D_Active_status_word_2__HEX_t;
    float CID_R_401F_CT_ratio_2_A_Signed_t;
    float CID_R_4021_Pulse_width_2_ms_Signed_t;
    float CID_R_4022_Pulse_type_setting_1_HEX_t;
    float CID_R_4023_Checksum_2_2__HEX_t;
    float CID_R_4026_Data_type_setting_1__Signed_t;
    float CID_R_4032_Screen_direction_1__Signed_t;
    float CID_R_4033_OBIS_code_1__Signed_t;
    float CID_R_5000_Voltage_2_V_Float_t;
    float CID_R_5002_L1_Voltage_2_V_Float_t;
    float CID_R_5004_L2_Voltage_2_V_Float_t;
    float CID_R_5006_L3_Voltage_2_V_Float_t;
    float CID_R_5008_Grid_frequency_2_Hz_Float_t;
    float CID_R_500A_Current_2_A_Float_t;
    float CID_R_500C_L1_Current_2_A_Float_t;
    float CID_R_500E_L2_Current_2_A_Float_t;
    float CID_R_5010_L3_Current_2_A_Float_t;
    float CID_R_5012_Total_active_power_2_kW_Float_t;
    float CID_R_5014_L1_Active_power_2_kW_Float_t;
    float CID_R_5016_L2_Active_power_2_kW_Float_t;
    float CID_R_5018_L3_Active_power_2_kW_Float_t;
    float CID_R_501A_Total_reactive_power_2_kvar_Float_t;
    float CID_R_501C_L1_Reactive_power_2_kvar_Float_t;
    float CID_R_501E_L2_Reactive_power_2_kvar_Float_t;
    float CID_R_5020_L3_Reactive_power_2_kvar_Float_t;
    float CID_R_5022_Total_apparent_power_2_kVA_Float_t;
    float CID_R_5024_L1_Apparent_power_2_kVA_Float_t;
    float CID_R_5026_L2_Apparent_power_2_kVA_Float_t;
    float CID_R_5028_L3_Apparent_power_2_kVA_Float_t;
    float CID_R_502A_Power_factor_2_Float_t;
    float CID_R_502C_L1_Power_factor_2_Float_t;
    float CID_R_502E_L2_Power_factor_2_Float_t;
    float CID_R_5030_L3_Power_factor_2_Float_t;
    float CID_R_5032_L1_L2_Voltage_2_V_Float_t;
    float CID_R_5034_L1_L3_Voltage_2_V_Float_t;
    float CID_R_5036_L2_L3_Voltage_2_V_Float_t;
    float CID_R_6000_Total_active_energy_2_kWh_Float_t;
    float CID_R_6002_T1_Total_active_energy_2_kWh_Float_t;
    float CID_R_6004_T2_Total_active_energy_2_kWh_Float_t;
    float CID_R_6006_L1_Total_active_energy_2_kWh_Float_t;
    float CID_R_6008_L2_Total_active_energy_2_kWh_Float_t;
    float CID_R_600A_L3_Total_active_energy_2_kWh_Float_t;
    float CID_R_600C_Forward_active_energy_2_kWh_Float_t;
    float CID_R_600E_T1_Forward_active_energy_2_kWh_Float_t;
    float CID_R_6010_T2_Forward_active_energy_2_kWh_Float_t;
    float CID_R_6012_L1_Forward_active_energy_2_kWh_Float_t;
    float CID_R_6014_L2_Forward_active_energy_2_kWh_Float_t;
    float CID_R_6016_L3_Forward_active_energy_2_kWh_Float_t;
    float CID_R_6018_Reverse_active_energy_2_kWh_Float_t;
    float CID_R_601A_T1_Reverse_active_energy_2_kWh_Float_t;
    float CID_R_601C_T2_Reverse_active_energy_2_kWh_Float_t;
    float CID_R_601E_L1_Reverse_active_energy_2_kWh_Float_t;
    float CID_R_6020_L2_Reverse_active_energy_2_kWh_Float_t;
    float CID_R_6022_L3_Reverse_active_energy_2_kWh_Float_t;
    float CID_R_6024_Total_reactive_energy_2_kvarh_Float_t;
    float CID_R_6026_T1_Total_reactive_energy_2_kvarh_Float_t;
    float CID_R_6028_T2_Total_reactive_energy_2_kvarh_Float_t;
    float CID_R_602A_L1_Total_reactive_energy_03_kvarh_Float_t;
    float CID_R_602C_L2_Total_reactive_energy_2_kvarh_Float_t;
    float CID_R_602E_L3_Total_reactive_energy_2_kvarh_Float_t;
    float CID_R_6030_Forward_reactive_energy_2_kvarh_Float_t;
    float CID_R_6032_T1_Forward_reactive_energy_2_kvarh_Float_t;
    float CID_R_6034_T2_Forward_reactive_energy_2_kvarh_Float_t;
    float CID_R_6036_L1_Forward_reactive_energy_2_kvarh_Float_t;
    float CID_R_6038_L2_Forward_reactive_energy_2_kvarh_Float_t;
    float CID_R_603A_L3_Forward_reactive_energy_2_kvarh_Float_t;
    float CID_R_603C_Reverse_reactive_energy_2_kvarh_Float_t;
    float CID_R_603E_T1_Reverse_reactive_energy_2_kvarh_Float_t;
    float CID_R_6040_T2_Reverse_reactive_energy_2_kvarh_Float_t;
    float CID_R_6042_L1_Reverse_reactive_energy_2_kvarh_Float_t;
    float CID_R_6044_L2_Reverse_reactive_energy_2_kvarh_Float_t;
    float CID_R_6046_L3_Reverse_reactive_energy_2_kvarh_Float_t;
    float CID_R_6048_Tariff_1__Signed_t;
    float CID_R_6049_Resettable_day_register_2_kWh_Float_t;
    float CID_R_604B_T3_Total_active_energy_2_kWh_Float_t;
    float CID_R_604D_T4_Total_active_energy_2_kWh_Float_t;
    float CID_R_604F_T3_Forward_active_energy_2_kWh_Float_t;
    float CID_R_6051_T4_Forward_active_energy_2_kWh_Float_t;
    float CID_R_6053_T3_Reverse_active_energy_2_kWh_Float_t;
    float CID_R_6055_T4_Reverse_active_energy_2_kWh_Float_t;
    float CID_R_6057_T3_Total_reactive_energy_2_kvarh_Float_t;
    float CID_R_6059_T4_Total_reactive_energy_2_kvarh_Float_t;
    float CID_R_605B_T3_Forward_reactive_energy_2_kvarh_Float_t;
    float CID_R_605D_T4_Forward_reactive_energy_2_kvarh_Float_t;
    float CID_R_605F_T3_Reverse_reactive_energy_2_kvarh_Float_t;
    float CID_R_6061_T4_Reverse_reactive_energy_2_kvarh_Float_t;
    float CID_R_6063_Imp_Inductive_reactive_energy_in_Q1_total_2_kWh_Float_t;
    float CID_R_6065_Imp_Inductive_reactive_energy_in_Q1_T1_2_kWh_Float_t;
    float CID_R_6067_Imp_Inductive_reactive_energy_in_Q1_T2_2_kWh_Float_t;
    float CID_R_6069_Imp_Inductive_reactive_energy_in_Q1_T3_2_kWh_Float_t;
    float CID_R_606B_Imp_Inductive_reactive_energy_in_Q1_T4_2_kWh_Float_t;
    float CID_R_606D_Imp_capacitive_reactive_energy_in_Q2_total_2_kWh_Float_t;
    float CID_R_606F_Imp_capacitive_reactive_energy_in_Q2_T1_2_kWh_Float_t;
    float CID_R_6071_Imp_capacitive_reactive_energy_in_Q2_T2_2_kWh_Float_t;
    float CID_R_6073_Imp_capacitive_reactive_energy_in_Q2_T3_03_2_kWh_Float_t;
    float CID_R_6075_Imp_capacitive_reactive_energy_in_Q2_T4_03_2_kWh_Float_t;
    float CID_R_6077_Exp_Inductive_reactive_energy_in_Q3_total_2_kWh_Float_t;
    float CID_R_6079_Exp_Inductive_reactive_energy_in_Q3_T1_2_kWh_Float_t;
    float CID_R_607B_Exp_Inductive_reactive_energy_in_Q3_T2_2_kWh_Float_t;
    float CID_R_607D_Exp_Inductive_reactive_energy_in_Q3_T3_2_kWh_Float_t;
    float CID_R_607F_Exp_Inductive_reactive_energy_in_Q3_T4_2_kWh_Float_t;
    float CID_R_6081_Exp_capacitive_reactive_energy_in_Q4_total_2_kWh_Float_t;
    float CID_R_6083_Exp_capacitive_reactive_energy_in_Q4_T1_2_kWh_Float_t;
    float CID_R_6085_Exp_capacitive_reactive_energy_in_Q4_T2_2_kWh_Float_t;
    float CID_R_6087_Exp_capacitive_reactive_energy_in_Q4_T3_2_kWh_Float_t;
    float CID_R_6089_Exp_capacitive_reactive_energy_in_Q4_T4_2_kWh_Float_t;
    float CID_R_608B_Resettable_day_counter_L1_2_kWh_Float_t;
    float CID_R_608D_Resettable_day_counter_L2_2_kWh_Float_t;
    float CID_R_608F_Resettable_day_counter_L3_2_kWh_Float_t;
}holding_reg_rw_params_t;

const unsigned char modbus_auchCRCHi[] =
{
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
    0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,
    0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,
    0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,
    0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
    0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,
    0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
    0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
    0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,
    0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
    0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
    0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,
    0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
    0x40
};

const unsigned char modbus_auchCRCLo[] =
{
    0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,0xC6,0x06,0x07,0xC7,0x05,0xC5,0xC4,
    0x04,0xCC,0x0C,0x0D,0xCD,0x0F,0xCF,0xCE,0x0E,0x0A,0xCA,0xCB,0x0B,0xC9,0x09,
    0x08,0xC8,0xD8,0x18,0x19,0xD9,0x1B,0xDB,0xDA,0x1A,0x1E,0xDE,0xDF,0x1F,0xDD,
    0x1D,0x1C,0xDC,0x14,0xD4,0xD5,0x15,0xD7,0x17,0x16,0xD6,0xD2,0x12,0x13,0xD3,
    0x11,0xD1,0xD0,0x10,0xF0,0x30,0x31,0xF1,0x33,0xF3,0xF2,0x32,0x36,0xF6,0xF7,
    0x37,0xF5,0x35,0x34,0xF4,0x3C,0xFC,0xFD,0x3D,0xFF,0x3F,0x3E,0xFE,0xFA,0x3A,
    0x3B,0xFB,0x39,0xF9,0xF8,0x38,0x28,0xE8,0xE9,0x29,0xEB,0x2B,0x2A,0xEA,0xEE,
    0x2E,0x2F,0xEF,0x2D,0xED,0xEC,0x2C,0xE4,0x24,0x25,0xE5,0x27,0xE7,0xE6,0x26,
    0x22,0xE2,0xE3,0x23,0xE1,0x21,0x20,0xE0,0xA0,0x60,0x61,0xA1,0x63,0xA3,0xA2,
    0x62,0x66,0xA6,0xA7,0x67,0xA5,0x65,0x64,0xA4,0x6C,0xAC,0xAD,0x6D,0xAF,0x6F,
    0x6E,0xAE,0xAA,0x6A,0x6B,0xAB,0x69,0xA9,0xA8,0x68,0x78,0xB8,0xB9,0x79,0xBB,
    0x7B,0x7A,0xBA,0xBE,0x7E,0x7F,0xBF,0x7D,0xBD,0xBC,0x7C,0xB4,0x74,0x75,0xB5,
    0x77,0xB7,0xB6,0x76,0x72,0xB2,0xB3,0x73,0xB1,0x71,0x70,0xB0,0x50,0x90,0x91,
    0x51,0x93,0x53,0x52,0x92,0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,0x9C,0x5C,
    0x5D,0x9D,0x5F,0x9F,0x9E,0x5E,0x5A,0x9A,0x9B,0x5B,0x99,0x59,0x58,0x98,0x88,
    0x48,0x49,0x89,0x4B,0x8B,0x8A,0x4A,0x4E,0x8E,0x8F,0x4F,0x8D,0x4D,0x4C,0x8C,
    0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,0x43,0x83,0x41,0x81,0x80,
    0x40
};

uint8_t modbus_try_cnt[CID_RW_COUNT];
bool modbus_operation_result[CID_RW_COUNT];
bool modbus_operation_enable[CID_RW_COUNT];
exception modbus_error_code[CID_RW_COUNT];

const modbus_operation_parameter_descriptor_t modbus_operation_parameters[] =
{
    { modbus_operation_enable, modbus_try_cnt, CID_R_4000_Serial_number_2__HEX, STR("Serial number"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4000, 2,
        HOLD_OFFSET_RW(CID_R_4000_Serial_number_2__HEX_t), PARAM_TYPE_HEX32, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4002_Meter_code_1__HEX, STR("Meter code"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4002, 1,
        HOLD_OFFSET_RW(CID_R_4002_Meter_code_1__HEX_t), PARAM_TYPE_HEX16, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4003_Modbus_ID_1__Signed, STR("Modbus ID"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4003, 1,
        HOLD_OFFSET_RW(CID_R_4003_Modbus_ID_1__Signed_t), PARAM_TYPE_U16, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4004_Baud_rate_1__Signed, STR("Baud rate"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4004, 1,
        HOLD_OFFSET_RW(CID_R_4004_Baud_rate_1__Signed_t), PARAM_TYPE_U16, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4005_Protocol_version_2__Float, STR("Protocol version"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4005, 2,
        HOLD_OFFSET_RW(CID_R_4005_Protocol_version_2__Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4007_Software_version_2__Float, STR("Software version"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4007, 2,
        HOLD_OFFSET_RW(CID_R_4007_Software_version_2__Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4009_Hardware_version_2__Float, STR("Hardware version"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4009, 2,
        HOLD_OFFSET_RW(CID_R_4009_Hardware_version_2__Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_400B_Meter_amps_1_A_Signed, STR("Meter amps"), STR("A"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x400B, 1,
        HOLD_OFFSET_RW(CID_R_400B_Meter_amps_1_A_Signed_t), PARAM_TYPE_U16, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_400C_CT_ratio_1_A_HEX, STR("CT ratio"), STR("A"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x400C, 1,
        HOLD_OFFSET_RW(CID_R_400C_CT_ratio_1_A_HEX_t), PARAM_TYPE_HEX16, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_400D_S0_output_rate_2_kWh_Float, STR("S0 output rate"), STR("pulse/kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x400D, 2,
        HOLD_OFFSET_RW(CID_R_400D_S0_output_rate_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_400F_Combination_code_1__HEX, STR("Combination code"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x400F, 1,
        HOLD_OFFSET_RW(CID_R_400F_Combination_code_1__HEX_t), PARAM_TYPE_U16, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4010_LCD_cycle_time_1_sec_HEX, STR("LCD cycle time 1 sec"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4010, 1,
        HOLD_OFFSET_RW(CID_R_4010_LCD_cycle_time_1_sec_HEX_t), PARAM_TYPE_HEX16, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4011_Parity_setting_1__Signed, STR("Parity setting"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4011, 1,
        HOLD_OFFSET_RW(CID_R_4011_Parity_setting_1__Signed_t), PARAM_TYPE_U16, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4012_Current_direction_1__ASCII, STR("Current direction"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4012, 1,
        HOLD_OFFSET_RW(CID_R_4012_Current_direction_1__ASCII_t), PARAM_TYPE_ASCII, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4013_L2_Current_direction_1__ASCII, STR("L2 Current direction"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4013, 1,
        HOLD_OFFSET_RW(CID_R_4013_L2_Current_direction_1__ASCII_t), PARAM_TYPE_ASCII, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4014_L3_Current_direction_1__ASCII, STR("L3 Current direction"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4014, 1,
        HOLD_OFFSET_RW(CID_R_4014_L3_Current_direction_1__ASCII_t), PARAM_TYPE_ASCII, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4016_Power_down_counter_1__Signed, STR("Power down counter"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4016, 1,
        HOLD_OFFSET_RW(CID_R_4016_Power_down_counter_1__Signed_t), PARAM_TYPE_U16, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4017_Present_quadrant_1__Signed, STR("Present quadrant"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4017, 1,
        HOLD_OFFSET_RW(CID_R_4017_Present_quadrant_1__Signed_t), PARAM_TYPE_U16, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4018_L1_Quadrant_1__Signed, STR("L1 quadrant"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4018, 1,
        HOLD_OFFSET_RW(CID_R_4018_L1_Quadrant_1__Signed_t), PARAM_TYPE_U16, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4019_L2_Quadrant_1__Signed, STR("L2 quadrant"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4019, 1,
        HOLD_OFFSET_RW(CID_R_4019_L2_Quadrant_1__Signed_t), PARAM_TYPE_U16, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_401A_L3_Quadrant_1__Signed, STR("L3 quadrant"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x401A, 1,
        HOLD_OFFSET_RW(CID_R_401A_L3_Quadrant_1__Signed_t), PARAM_TYPE_U16, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_401B_Checksum_2__HEX, STR("Checksum"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x401B, 2,
        HOLD_OFFSET_RW(CID_R_401B_Checksum_2__HEX_t), PARAM_TYPE_HEX32, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_401D_Active_status_word_2__HEX, STR("Active status word"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x401D, 2,
        HOLD_OFFSET_RW(CID_R_401D_Active_status_word_2__HEX_t), PARAM_TYPE_HEX32, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_401F_CT_ratio_2_A_Signed, STR("CT ratio 2"), STR("A"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x401F, 2,
        HOLD_OFFSET_RW(CID_R_401F_CT_ratio_2_A_Signed_t), PARAM_TYPE_BIN32, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4021_Pulse_width_2_ms_Signed, STR("Pulse width"), STR("ms"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4021, 2,
        HOLD_OFFSET_RW(CID_R_4021_Pulse_width_2_ms_Signed_t), PARAM_TYPE_BIN32, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4022_Pulse_type_setting_1_HEX, STR("Pulse type"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4022, 1,
        HOLD_OFFSET_RW(CID_R_4022_Pulse_type_setting_1_HEX_t), PARAM_TYPE_HEX16, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4023_Checksum_2_2__HEX, STR("Checksum 2"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4023, 2,
        HOLD_OFFSET_RW(CID_R_4023_Checksum_2_2__HEX_t), PARAM_TYPE_HEX32, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4026_Data_type_setting_1__Signed, STR("Data type setting"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4026, 1,
        HOLD_OFFSET_RW(CID_R_4026_Data_type_setting_1__Signed_t), PARAM_TYPE_U16, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4032_Screen_direction_1__Signed, STR("Screen direction"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4032, 1,
        HOLD_OFFSET_RW(CID_R_4032_Screen_direction_1__Signed_t), PARAM_TYPE_U16, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_4033_OBIS_code_1__Signed, STR("OBIS code"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4033, 1,
        HOLD_OFFSET_RW(CID_R_4033_OBIS_code_1__Signed_t), PARAM_TYPE_U16, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_5000_Voltage_2_V_Float, STR("Voltage"), STR("V"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5000, 2,
        HOLD_OFFSET_RW(CID_R_5000_Voltage_2_V_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code},
    { modbus_operation_enable, modbus_try_cnt, CID_R_5002_L1_Voltage_2_V_Float, STR("L1 Voltage"), STR("V"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5002, 2,
        HOLD_OFFSET_RW(CID_R_5002_L1_Voltage_2_V_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_5004_L2_Voltage_2_V_Float, STR("L2 Voltage"), STR("V"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5004, 2,
        HOLD_OFFSET_RW(CID_R_5004_L2_Voltage_2_V_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_5006_L3_Voltage_2_V_Float, STR("L3 Voltage"), STR("V"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5006, 2,
        HOLD_OFFSET_RW(CID_R_5006_L3_Voltage_2_V_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_5008_Grid_frequency_2_Hz_Float, STR("Grid frequency"), STR("Hz"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5008, 2,
        HOLD_OFFSET_RW(CID_R_5008_Grid_frequency_2_Hz_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_500A_Current_2_A_Float, STR("Current"), STR("A"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x500A, 2,
        HOLD_OFFSET_RW(CID_R_500A_Current_2_A_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_500C_L1_Current_2_A_Float, STR("L1 Current"), STR("A"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x500C, 2,
        HOLD_OFFSET_RW(CID_R_500C_L1_Current_2_A_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_500E_L2_Current_2_A_Float, STR("L2 Current"), STR("A"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x500E, 2,
        HOLD_OFFSET_RW(CID_R_500E_L2_Current_2_A_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_5010_L3_Current_2_A_Float, STR("L3 Current"), STR("A"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5010, 2,
        HOLD_OFFSET_RW(CID_R_5010_L3_Current_2_A_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_5012_Total_active_power_2_kW_Float, STR("Total active power"), STR("kW"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5012, 2,
        HOLD_OFFSET_RW(CID_R_5012_Total_active_power_2_kW_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_5014_L1_Active_power_2_kW_Float, STR("L1 Active power"), STR("kW"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5014, 2,
        HOLD_OFFSET_RW(CID_R_5014_L1_Active_power_2_kW_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_5016_L2_Active_power_2_kW_Float, STR("L2 Active power"), STR("kW"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5016, 2,
        HOLD_OFFSET_RW(CID_R_5016_L2_Active_power_2_kW_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_5018_L3_Active_power_2_kW_Float, STR("L3 Active power"), STR("kW"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5018, 2,
        HOLD_OFFSET_RW(CID_R_5018_L3_Active_power_2_kW_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_501A_Total_reactive_power_2_kvar_Float, STR("Total reactive power"), STR("kvar"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x501A, 2,
        HOLD_OFFSET_RW(CID_R_501A_Total_reactive_power_2_kvar_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_501C_L1_Reactive_power_2_kvar_Float, STR("L1 reactive power"), STR("kvar"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x501C, 2,
        HOLD_OFFSET_RW(CID_R_501C_L1_Reactive_power_2_kvar_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_501E_L2_Reactive_power_2_kvar_Float, STR("L2 reactive power"), STR("kvar"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x501E, 2,
        HOLD_OFFSET_RW(CID_R_501E_L2_Reactive_power_2_kvar_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_5020_L3_Reactive_power_2_kvar_Float, STR("L3 reactive power"), STR("kvar"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5020, 2,
        HOLD_OFFSET_RW(CID_R_5020_L3_Reactive_power_2_kvar_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_5022_Total_apparent_power_2_kVA_Float, STR("Total apparent power"), STR("kVA"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5022, 2,
        HOLD_OFFSET_RW(CID_R_5022_Total_apparent_power_2_kVA_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_5024_L1_Apparent_power_2_kVA_Float, STR("L1 apparent power"), STR("kVA"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5024, 2,
        HOLD_OFFSET_RW(CID_R_5024_L1_Apparent_power_2_kVA_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_5026_L2_Apparent_power_2_kVA_Float, STR("L2 apparent power"), STR("kVA"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5026, 2,
        HOLD_OFFSET_RW(CID_R_5026_L2_Apparent_power_2_kVA_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_5028_L3_Apparent_power_2_kVA_Float, STR("L3 apparent power"), STR("kVA"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5028, 2,
        HOLD_OFFSET_RW(CID_R_5028_L3_Apparent_power_2_kVA_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_502A_Power_factor_2_Float, STR("Power factor"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x502A, 2,
        HOLD_OFFSET_RW(CID_R_502A_Power_factor_2_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_502C_L1_Power_factor_2_Float, STR("L1 Power factor"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x502C, 2,
        HOLD_OFFSET_RW(CID_R_502C_L1_Power_factor_2_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_502E_L2_Power_factor_2_Float, STR("L2 Power factor"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x502E, 2,
        HOLD_OFFSET_RW(CID_R_502E_L2_Power_factor_2_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_5030_L3_Power_factor_2_Float, STR("L3 Power factor"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5030, 2,
        HOLD_OFFSET_RW(CID_R_5030_L3_Power_factor_2_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_5032_L1_L2_Voltage_2_V_Float, STR("L1 L2 Voltage"), STR("V"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5032, 2,
        HOLD_OFFSET_RW(CID_R_5032_L1_L2_Voltage_2_V_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_5034_L1_L3_Voltage_2_V_Float, STR("L1 L3 Voltage"), STR("V"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5034, 2,
        HOLD_OFFSET_RW(CID_R_5034_L1_L3_Voltage_2_V_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_5036_L2_L3_Voltage_2_V_Float, STR("L2 L3 Voltage"), STR("V"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x5036, 2,
        HOLD_OFFSET_RW(CID_R_5036_L2_L3_Voltage_2_V_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6000_Total_active_energy_2_kWh_Float, STR("Total active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6000, 2,
        HOLD_OFFSET_RW(CID_R_6000_Total_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6002_T1_Total_active_energy_2_kWh_Float, STR("T1_Total active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6002, 2,
        HOLD_OFFSET_RW(CID_R_6002_T1_Total_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6004_T2_Total_active_energy_2_kWh_Float, STR("T1 Total active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6004, 2,
        HOLD_OFFSET_RW(CID_R_6004_T2_Total_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6006_L1_Total_active_energy_2_kWh_Float, STR("L1 Total active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6006, 2,
        HOLD_OFFSET_RW(CID_R_6006_L1_Total_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6008_L2_Total_active_energy_2_kWh_Float, STR("L2 Total active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6008, 2,
        HOLD_OFFSET_RW(CID_R_6008_L2_Total_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_600A_L3_Total_active_energy_2_kWh_Float, STR("L3 Total active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x600A, 2,
        HOLD_OFFSET_RW(CID_R_600A_L3_Total_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_600C_Forward_active_energy_2_kWh_Float, STR("Forward active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x600C, 2,
        HOLD_OFFSET_RW(CID_R_600C_Forward_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_600E_T1_Forward_active_energy_2_kWh_Float, STR("T1 Forward active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x600E, 2,
        HOLD_OFFSET_RW(CID_R_600E_T1_Forward_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6010_T2_Forward_active_energy_2_kWh_Float, STR("T2 Total active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6010, 2,
        HOLD_OFFSET_RW(CID_R_6010_T2_Forward_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6012_L1_Forward_active_energy_2_kWh_Float, STR("L1 Forward active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6012, 2,
        HOLD_OFFSET_RW(CID_R_6012_L1_Forward_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6014_L2_Forward_active_energy_2_kWh_Float, STR("L2 Forward active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6014, 2,
        HOLD_OFFSET_RW(CID_R_6014_L2_Forward_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6016_L3_Forward_active_energy_2_kWh_Float, STR("L3 Forward active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6016, 2,
        HOLD_OFFSET_RW(CID_R_6016_L3_Forward_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6018_Reverse_active_energy_2_kWh_Float, STR("Reverse active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6018, 2,
        HOLD_OFFSET_RW(CID_R_6018_Reverse_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_601A_T1_Reverse_active_energy_2_kWh_Float, STR("T1 Reverse active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x601A, 2,
        HOLD_OFFSET_RW(CID_R_601A_T1_Reverse_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_601C_T2_Reverse_active_energy_2_kWh_Float, STR("T2 Reverse active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x601C, 2,
        HOLD_OFFSET_RW(CID_R_601C_T2_Reverse_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_601E_L1_Reverse_active_energy_2_kWh_Float, STR("L1 Reverse active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x601E, 2,
        HOLD_OFFSET_RW(CID_R_601E_L1_Reverse_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6020_L2_Reverse_active_energy_2_kWh_Float, STR("L2 Reverse active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6020, 2,
        HOLD_OFFSET_RW(CID_R_6020_L2_Reverse_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6022_L3_Reverse_active_energy_2_kWh_Float, STR("L3 Reverse active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6022, 2,
        HOLD_OFFSET_RW(CID_R_6022_L3_Reverse_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6024_Total_reactive_energy_2_kvarh_Float, STR("Total reactive energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6024, 2,
        HOLD_OFFSET_RW(CID_R_6024_Total_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6026_T1_Total_reactive_energy_2_kvarh_Float, STR("T1 Total reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6026, 2,
        HOLD_OFFSET_RW(CID_R_6026_T1_Total_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6028_T2_Total_reactive_energy_2_kvarh_Float, STR("T2 Total reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6028, 2,
        HOLD_OFFSET_RW(CID_R_6028_T2_Total_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_602A_L1_Total_reactive_energy_03_kvarh_Float, STR("L1 Total reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x602A, 2,
        HOLD_OFFSET_RW(CID_R_602A_L1_Total_reactive_energy_03_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_602C_L2_Total_reactive_energy_2_kvarh_Float, STR("L2 Total reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x602C, 2,
        HOLD_OFFSET_RW(CID_R_602C_L2_Total_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_602E_L3_Total_reactive_energy_2_kvarh_Float, STR("L3 Total reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x602E, 2,
        HOLD_OFFSET_RW(CID_R_602E_L3_Total_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6030_Forward_reactive_energy_2_kvarh_Float, STR("Forward reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6030, 2,
        HOLD_OFFSET_RW(CID_R_6030_Forward_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6032_T1_Forward_reactive_energy_2_kvarh_Float, STR("T1 Forward reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6032, 2,
        HOLD_OFFSET_RW(CID_R_6032_T1_Forward_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6034_T2_Forward_reactive_energy_2_kvarh_Float, STR("T2 Forward reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6034, 2,
        HOLD_OFFSET_RW(CID_R_6034_T2_Forward_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6036_L1_Forward_reactive_energy_2_kvarh_Float, STR("L1 Forward reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6036, 2,
        HOLD_OFFSET_RW(CID_R_6036_L1_Forward_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6038_L2_Forward_reactive_energy_2_kvarh_Float, STR("L2 Forward reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6038, 2,
        HOLD_OFFSET_RW(CID_R_6038_L2_Forward_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_603A_L3_Forward_reactive_energy_2_kvarh_Float, STR("L3 Forward reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x603A, 2,
        HOLD_OFFSET_RW(CID_R_603A_L3_Forward_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_603C_Reverse_reactive_energy_2_kvarh_Float, STR("Reverse reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x603C, 2,
        HOLD_OFFSET_RW(CID_R_603C_Reverse_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_603E_T1_Reverse_reactive_energy_2_kvarh_Float, STR("T1 Reverse reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x603E, 2,
        HOLD_OFFSET_RW(CID_R_603E_T1_Reverse_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6040_T2_Reverse_reactive_energy_2_kvarh_Float, STR("T2 Reverse reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x603E, 2,
        HOLD_OFFSET_RW(CID_R_6040_T2_Reverse_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6042_L1_Reverse_reactive_energy_2_kvarh_Float, STR("L1 Reverse reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6042, 2,
        HOLD_OFFSET_RW(CID_R_6042_L1_Reverse_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6044_L2_Reverse_reactive_energy_2_kvarh_Float, STR("L2 Reverse reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6044, 2,
        HOLD_OFFSET_RW(CID_R_6044_L2_Reverse_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6046_L3_Reverse_reactive_energy_2_kvarh_Float, STR("L3 Reverse reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6046, 2,
        HOLD_OFFSET_RW(CID_R_6046_L3_Reverse_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6048_Tariff_1__Signed, STR("Tariff"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6048, 1,
        HOLD_OFFSET_RW(CID_R_6048_Tariff_1__Signed_t), PARAM_TYPE_U16, PARAM_SIZE_U16, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6049_Resettable_day_register_2_kWh_Float, STR("Resettable day register"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6049, 2,
        HOLD_OFFSET_RW(CID_R_6049_Resettable_day_register_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_604B_T3_Total_active_energy_2_kWh_Float, STR("T3 Total active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x604B, 2,
        HOLD_OFFSET_RW(CID_R_604B_T3_Total_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_604D_T4_Total_active_energy_2_kWh_Float, STR("T4 Total active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x604D, 2,
        HOLD_OFFSET_RW(CID_R_604D_T4_Total_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_604F_T3_Forward_active_energy_2_kWh_Float, STR("T3 Forward active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x604F, 2,
        HOLD_OFFSET_RW(CID_R_604F_T3_Forward_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6051_T4_Forward_active_energy_2_kWh_Float, STR("T4 Forward active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6051, 2,
        HOLD_OFFSET_RW(CID_R_6051_T4_Forward_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6053_T3_Reverse_active_energy_2_kWh_Float, STR("T3 Reverse active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6053, 2,
        HOLD_OFFSET_RW(CID_R_6053_T3_Reverse_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6055_T4_Reverse_active_energy_2_kWh_Float, STR("T4 Reverse active energy"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6055, 2,
        HOLD_OFFSET_RW(CID_R_6055_T4_Reverse_active_energy_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6057_T3_Total_reactive_energy_2_kvarh_Float, STR("T3 Total reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6057, 2,
        HOLD_OFFSET_RW(CID_R_6057_T3_Total_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6059_T4_Total_reactive_energy_2_kvarh_Float, STR("T4 Total reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6059, 2,
        HOLD_OFFSET_RW(CID_R_6059_T4_Total_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_605B_T3_Forward_reactive_energy_2_kvarh_Float, STR("T3 Forward reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x605B, 2,
        HOLD_OFFSET_RW(CID_R_605B_T3_Forward_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_605D_T4_Forward_reactive_energy_2_kvarh_Float, STR("T4 Forward reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x605D, 2,
        HOLD_OFFSET_RW(CID_R_605D_T4_Forward_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_605F_T3_Reverse_reactive_energy_2_kvarh_Float, STR("T3 Reverse reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x605F, 2,
        HOLD_OFFSET_RW(CID_R_605F_T3_Reverse_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6061_T4_Reverse_reactive_energy_2_kvarh_Float, STR("T4 Reverse reactive energy"), STR("kvarh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6061, 2,
        HOLD_OFFSET_RW(CID_R_6061_T4_Reverse_reactive_energy_2_kvarh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6063_Imp_Inductive_reactive_energy_in_Q1_total_2_kWh_Float, STR("Imp. inductive reactive energy in Q1 (Total)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6063, 2,
        HOLD_OFFSET_RW(CID_R_6063_Imp_Inductive_reactive_energy_in_Q1_total_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6065_Imp_Inductive_reactive_energy_in_Q1_T1_2_kWh_Float, STR("Imp. inductive reactive energy in Q1 (T1)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6065, 2,
        HOLD_OFFSET_RW(CID_R_6065_Imp_Inductive_reactive_energy_in_Q1_T1_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6067_Imp_Inductive_reactive_energy_in_Q1_T2_2_kWh_Float, STR("Imp. inductive reactive energy in Q1 (T2)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6067, 2,
        HOLD_OFFSET_RW(CID_R_6067_Imp_Inductive_reactive_energy_in_Q1_T2_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6069_Imp_Inductive_reactive_energy_in_Q1_T3_2_kWh_Float, STR("Imp. inductive reactive energy in Q1 (T3)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6069, 2,
        HOLD_OFFSET_RW(CID_R_6069_Imp_Inductive_reactive_energy_in_Q1_T3_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_606B_Imp_Inductive_reactive_energy_in_Q1_T4_2_kWh_Float, STR("Imp. inductive reactive energy in Q1 (T4)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x606B, 2,
        HOLD_OFFSET_RW(CID_R_606B_Imp_Inductive_reactive_energy_in_Q1_T4_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_606D_Imp_capacitive_reactive_energy_in_Q2_total_2_kWh_Float, STR("Imp. capacitive reactive energy in Q2 (Total)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x606B, 2,
        HOLD_OFFSET_RW(CID_R_606D_Imp_capacitive_reactive_energy_in_Q2_total_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_606F_Imp_capacitive_reactive_energy_in_Q2_T1_2_kWh_Float, STR("Imp. capacitive reactive energy in Q1 (T1)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x606F, 2,
        HOLD_OFFSET_RW(CID_R_606F_Imp_capacitive_reactive_energy_in_Q2_T1_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6071_Imp_capacitive_reactive_energy_in_Q2_T2_2_kWh_Float, STR("Imp. capacitive reactive energy in Q1 (T2)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6071, 2,
        HOLD_OFFSET_RW(CID_R_6071_Imp_capacitive_reactive_energy_in_Q2_T2_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6073_Imp_capacitive_reactive_energy_in_Q2_T3_03_2_kWh_Float, STR("Imp. capacitive reactive energy in Q1 (T3)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6073, 2,
        HOLD_OFFSET_RW(CID_R_6073_Imp_capacitive_reactive_energy_in_Q2_T3_03_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6075_Imp_capacitive_reactive_energy_in_Q2_T4_03_2_kWh_Float, STR("Imp. capacitive reactive energy in Q2 (T4)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6075, 2,
        HOLD_OFFSET_RW(CID_R_6075_Imp_capacitive_reactive_energy_in_Q2_T4_03_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6077_Exp_Inductive_reactive_energy_in_Q3_total_2_kWh_Float, STR("Exp. Inductive reactive energy in Q3 (Total)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6077, 2,
        HOLD_OFFSET_RW(CID_R_6077_Exp_Inductive_reactive_energy_in_Q3_total_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6079_Exp_Inductive_reactive_energy_in_Q3_T1_2_kWh_Float, STR("Exp. Inductive reactive energy in Q3 (T1)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6079, 2,
        HOLD_OFFSET_RW(CID_R_6079_Exp_Inductive_reactive_energy_in_Q3_T1_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_607B_Exp_Inductive_reactive_energy_in_Q3_T2_2_kWh_Float, STR("Exp. Inductive reactive energy in Q3 (T2)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x607B, 2,
        HOLD_OFFSET_RW(CID_R_607B_Exp_Inductive_reactive_energy_in_Q3_T2_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_607D_Exp_Inductive_reactive_energy_in_Q3_T3_2_kWh_Float, STR("Exp. Inductive reactive energy in Q3 (T3)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x607D, 2,
        HOLD_OFFSET_RW(CID_R_607D_Exp_Inductive_reactive_energy_in_Q3_T3_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_607F_Exp_Inductive_reactive_energy_in_Q3_T4_2_kWh_Float, STR("Exp. Inductive reactive energy in Q3 (T4)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x607F, 2,
        HOLD_OFFSET_RW(CID_R_607F_Exp_Inductive_reactive_energy_in_Q3_T4_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6081_Exp_capacitive_reactive_energy_in_Q4_total_2_kWh_Float, STR("Exp. capacitive reactive energy in Q4 (Total)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6081, 2,
        HOLD_OFFSET_RW(CID_R_6081_Exp_capacitive_reactive_energy_in_Q4_total_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6083_Exp_capacitive_reactive_energy_in_Q4_T1_2_kWh_Float, STR("Exp. capacitive reactive energy in Q4 (T1)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6083, 2,
        HOLD_OFFSET_RW(CID_R_6083_Exp_capacitive_reactive_energy_in_Q4_T1_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6085_Exp_capacitive_reactive_energy_in_Q4_T2_2_kWh_Float, STR("Exp. capacitive reactive energy in Q4 (T2)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6085, 2,
        HOLD_OFFSET_RW(CID_R_6085_Exp_capacitive_reactive_energy_in_Q4_T2_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6087_Exp_capacitive_reactive_energy_in_Q4_T3_2_kWh_Float, STR("Exp. capacitive reactive energy in Q4 (T3)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6087, 2,
        HOLD_OFFSET_RW(CID_R_6087_Exp_capacitive_reactive_energy_in_Q4_T3_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_6089_Exp_capacitive_reactive_energy_in_Q4_T4_2_kWh_Float, STR("Exp. capacitive reactive energy in Q4 (T4)"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x6089, 2,
        HOLD_OFFSET_RW(CID_R_6089_Exp_capacitive_reactive_energy_in_Q4_T4_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_608B_Resettable_day_counter_L1_2_kWh_Float, STR("Resettable day counter L1"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x608B, 2,
        HOLD_OFFSET_RW(CID_R_608B_Resettable_day_counter_L1_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_608D_Resettable_day_counter_L2_2_kWh_Float, STR("Resettable day counter L2"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x608D, 2,
        HOLD_OFFSET_RW(CID_R_608D_Resettable_day_counter_L2_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
    { modbus_operation_enable, modbus_try_cnt, CID_R_608F_Resettable_day_counter_L3_2_kWh_Float, STR("Resettable day counter L3"), STR("kWh"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x608F, 2,
        HOLD_OFFSET_RW(CID_R_608F_Resettable_day_counter_L3_2_kWh_Float_t), PARAM_TYPE_FLOAT, PARAM_SIZE_U32, PAR_PERMS_READ, modbus_operation_result, modbus_error_code },
};

TaskHandle_t TaskHandle_uart1_modbus_rx_task = NULL;
static const int RX_BUF_SIZE = 1024;
_modbus_serial_state modbus_serial_state;
uint8_t dt_cnt;
uint8_t imm_crc[2];
exception temp_err;
modbus_dt_struct modbus_rx;
uint32_t modbus_error;
uint32_t modbus_timeout;
_enum_internal_modbus_operation enum_internal_modbus_operation = MODBUS_ITERATE_CID;
unsigned long modbus_get_timestamp;
holding_reg_rw_params_t holding_reg_rw_params = { 0 };
char modbus_write_str[100];
_enum_fpm_modbus_write enum_modbus_write = MODBUSWRITE_DEFAULT;
uint16_t cid = 0;
const uint16_t cid_operation_count = (sizeof(modbus_operation_parameters) / sizeof(modbus_operation_parameters[0]));

const char *TAG = "MODBUS";

void modbus_uart_init(int txd, int rxd, int baud, uart_parity_t parity)
{
    static uart_config_t uart_config;
    uart_config.baud_rate = baud;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = parity;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    uart_driver_install(MODBUS_UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(MODBUS_UART_NUM, &uart_config);
    uart_set_pin(MODBUS_UART_NUM, txd, rxd, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static int send_uart1(uint8_t* data, uint16_t len)
{
    const int txBytes = uart_write_bytes(MODBUS_UART_NUM, data, len);
    return txBytes;
}

static void *master_get_param_data(const modbus_operation_parameter_descriptor_t *operation_descriptor)
{
    assert(operation_descriptor != NULL);
    void *instance_ptr = NULL;
    switch (operation_descriptor->mb_param_type)
    {
        case MB_PARAM_HOLDING:
            if(operation_descriptor->access == PAR_PERMS_READ)
            {
                instance_ptr = ((void *)&holding_reg_rw_params + operation_descriptor->param_offset);
            }
            break;
        default:
            instance_ptr = NULL;
            break;
    }
    return instance_ptr;
}

static uint8_t make8(uint16_t reg, bool loc)
{
    static uint16_t _reg;
    _reg = reg;
	if(loc == 1)
	{
		return (uint8_t)((_reg>>8) & 0x00FF);
	}
	else
	{
		return (uint8_t)(_reg & 0x00FF);
	}
}

static void modbus_calc_crc(uint8_t data)
{
	static uint8_t uIndex ;
	uIndex = (modbus_serial_crc.b[1]) ^ data; // calculate the CRC
	modbus_serial_crc.b[1] = (modbus_serial_crc.b[0]) ^ modbus_auchCRCHi[uIndex];
	modbus_serial_crc.b[0] = modbus_auchCRCLo[uIndex];
	uIndex = 0;
}

static void modbus_char_delay(uint32_t delay)
{
    if(delay == T1_5)
    {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    else if(delay == T3_5)
    {
        vTaskDelay(pdMS_TO_TICKS(4));
    }
}

static void modbus_serial_putc(uint8_t dt)
{
    static uint8_t c;
    c = dt;
    modbus_calc_crc(c);
    modbus_char_delay(T1_5);
    send_uart1(&c, 1);
}

static void modbus_serial_send_start(uint8_t to, uint8_t func)
{
	modbus_serial_putc(to);
	modbus_serial_putc(func);
}

static void modbus_serial_send_crc()
{
	static uint8_t crc_low, crc_high;
	crc_high=modbus_serial_crc.b[1];
	crc_low=modbus_serial_crc.b[0];
	modbus_serial_putc(crc_high);
	modbus_serial_putc(crc_low);
}

static void init_modbus_rw(void)
{
	memset(&modbus_rx, 0, sizeof(modbus_rx));
	modbus_serial_crc.d=0xFFFF;//reset crc
	modbus_rx.error_code[0] = 0;
    imm_crc[1] = 0;
	imm_crc[0] = 0;
    modbus_timeout = MODBUS_GET_TIMEOUT;
	modbus_char_delay(T3_5);
    modbus_serial_state = MODBUS_GETADDY;
}

static void modbus_read_holding_registers(uint8_t address, uint16_t start_address, uint16_t quantity)
{
   modbus_serial_send_start(address, FUNC_READ_HOLDING_REGISTERS);
   modbus_serial_putc(make8(start_address,1));
   modbus_serial_putc(make8(start_address,0));
   modbus_serial_putc(make8(quantity,1));
   modbus_serial_putc(make8(quantity,0));
   modbus_serial_send_crc();
}

static void incomming_modbus_serial_READ(uint8_t chr)
{
    modbus_get_timestamp = xTaskGetTickCount();
	if(modbus_serial_state == MODBUS_GETADDY)
	{
		modbus_rx.error_code[1] = SERIAL_MALF;
		modbus_rx.address = chr;
		modbus_serial_state = MODBUS_GETFUNC;
		modbus_calc_crc(chr);
	}
	else if(modbus_serial_state == MODBUS_GETFUNC)
	{
		modbus_rx.func = chr;
		modbus_serial_state = MODBUS_GETLEN;
		modbus_calc_crc(chr);
	}
	else if(modbus_serial_state == MODBUS_GETLEN)
	{
		modbus_rx.error_code[0] = chr;
		modbus_rx.len = chr;
		modbus_serial_state = MODBUS_GETDATA;
		dt_cnt = 0;
		modbus_calc_crc(chr);
		imm_crc[1] = modbus_serial_crc.b[1];
		imm_crc[0] = modbus_serial_crc.b[0];
	}
	else if(modbus_serial_state == MODBUS_GETDATA)
	{
		if (modbus_rx.len>=MODBUS_SERIAL_RX_BUFFER_SIZE)
		{
			modbus_rx.len=MODBUS_SERIAL_RX_BUFFER_SIZE;
		}

		modbus_rx.data[dt_cnt]=chr;
		dt_cnt++;
		modbus_calc_crc(chr);
		modbus_rx.error_code[1] = SERIAL_MALF;

		if(dt_cnt==2)
		{
			if((imm_crc[1] != modbus_rx.data[0])||(imm_crc[0] != modbus_rx.data[1]))
			{
				modbus_rx.error_code[1] = CRC_MISM;
			}
			else
			{
				modbus_rx.error_code[1] = 0;
			}
		}

		if(dt_cnt>=(modbus_rx.len))
		{
			modbus_serial_state = MODBUS_GETCRC;
			dt_cnt=0;
		}
	}
	else if(modbus_serial_state == MODBUS_GETCRC)
	{
		modbus_rx.error_code[1] = SERIAL_MALF;
		modbus_rx.crc[dt_cnt] = chr;
		dt_cnt++;
		if(dt_cnt == 2)
		{
			modbus_rx.error_code[0] = 0;
			modbus_rx.error_code[1] = 0;
			modbus_serial_state = MODBUS_RXCOMPLETE;
			if((modbus_rx.crc[0] != modbus_serial_crc.b[1])||(modbus_rx.crc[1] != modbus_serial_crc.b[0]))
			{
				modbus_rx.error_code[1] = CRC_MISM;
			}
		}
	}
	else if(modbus_serial_state == MODBUS_RXCOMPLETE)
	{
		modbus_rx.error_code[1] = SERIAL_MALF;
	}
}

static void incomming_modbus_serial_WRITE_GENERIC(uint8_t chr)
{
    modbus_get_timestamp = xTaskGetTickCount();
	if(modbus_serial_state == MODBUS_GETADDY)
	{
		modbus_rx.error_code[1] = SERIAL_MALF;
		modbus_rx.address = chr;
		modbus_serial_state = MODBUS_GETFUNC;
		modbus_calc_crc(chr);
	}
	else if(modbus_serial_state == MODBUS_GETFUNC)
	{
		modbus_rx.func = chr;
		modbus_serial_state = MODBUS_GET_REGISTER_ADDRESS;
        dt_cnt = 0;
		modbus_calc_crc(chr);
	}
    else if(modbus_serial_state == MODBUS_GET_REGISTER_ADDRESS)
	{
        modbus_rx.crc[0] = chr;
        if(dt_cnt == 0)
        {
            modbus_rx.error_code[0] = chr;
        }
        modbus_rx.add[dt_cnt]=chr;
        dt_cnt++;
        modbus_calc_crc(chr);
        if(dt_cnt == 2)
        {
            dt_cnt = 0;
            if(modbus_rx.func == FUNC_WRITE_SINGLE_REGISTER)
            {
                modbus_serial_state = MODBUS_GETDATA;
            }
            else if(modbus_rx.func == FUNC_WRITE_MULTIPLE_REGISTERS)
            {
                modbus_serial_state = MODBUS_GETQUANTITY;
            }
            else
            {
                modbus_serial_state = MODBUS_WAIT_TIMEOUT;
            }
        }
	}
	else if(modbus_serial_state == MODBUS_GETDATA)
	{
        modbus_rx.crc[1] = chr;
        modbus_rx.data[dt_cnt]=chr;
        dt_cnt++;
        modbus_calc_crc(chr);
        if(dt_cnt == 1)
        {
        	imm_crc[1] = modbus_serial_crc.b[1];
		    imm_crc[0] = modbus_serial_crc.b[0];
            if((modbus_rx.crc[0] != modbus_serial_crc.b[1])||(modbus_rx.crc[1] != modbus_serial_crc.b[0]))
			{
				modbus_rx.error_code[1] = CRC_MISM;
			}
        }
        if(dt_cnt == 2)
        {
            modbus_serial_state = MODBUS_GETCRC;
            dt_cnt = 0;
        }
	}
    else if(modbus_serial_state == MODBUS_GETQUANTITY)
	{
        modbus_rx.crc[1] = chr;
        modbus_rx.qty[dt_cnt] = chr;
        dt_cnt++;
        modbus_calc_crc(chr);
        if(dt_cnt == 1)
        {
        	imm_crc[1] = modbus_serial_crc.b[1];
		    imm_crc[0] = modbus_serial_crc.b[0];
            if((modbus_rx.crc[0] != modbus_serial_crc.b[1])||(modbus_rx.crc[1] != modbus_serial_crc.b[0]))
			{
				modbus_rx.error_code[1] = CRC_MISM;
			}
        }
        if(dt_cnt == 2)
        {
            modbus_serial_state = MODBUS_GETCRC;
            dt_cnt = 0;
        }
	}
	else if(modbus_serial_state == MODBUS_GETCRC)
	{
		modbus_rx.error_code[1] = SERIAL_MALF;
		modbus_rx.crc[dt_cnt] = chr;
		dt_cnt++;
		if(dt_cnt == 2)
		{
			modbus_rx.error_code[0] = 0;
			modbus_rx.error_code[1] = 0;
			modbus_serial_state = MODBUS_RXCOMPLETE;
			if((modbus_rx.crc[0] != modbus_serial_crc.b[1])||(modbus_rx.crc[1] != modbus_serial_crc.b[0]))
			{
				modbus_rx.error_code[1] = CRC_MISM;
			}
		}
	}
	else if(modbus_serial_state == MODBUS_RXCOMPLETE)
	{
		modbus_rx.error_code[1] = SERIAL_MALF;
	}
    else if(modbus_serial_state == MODBUS_WAIT_TIMEOUT)
	{
	}
    
}

void read_modbus_uart()
{
    static int rxBytes;
    static uint8_t data[1024];
    static uint16_t data_idx;
    rxBytes = uart_read_bytes(MODBUS_UART_NUM, data, RX_BUF_SIZE, 5 / portTICK_PERIOD_MS);
    data_idx = 0;
    while(rxBytes) 
    {
        if(enum_internal_modbus_operation == MODBUS_READ_WAIT)
        {
            incomming_modbus_serial_READ(data[data_idx]);
        }
        else if(enum_internal_modbus_operation == MODBUS_WRITE_GENERIC_WAIT)
        {
            incomming_modbus_serial_WRITE_GENERIC(data[data_idx]);
        }
        data_idx++;
        rxBytes--;
    }
}
static void uart1_modbus_rx_task(void *arg)
{
    while (1)
    {
        read_modbus_uart();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void start_modbus_uart_task(void)
{
    modbus_uart_init(MODBUS_TXD_PIN, MODBUS_RXD_PIN, 115200, UART_PARITY_EVEN);
    xTaskCreatePinnedToCore(uart1_modbus_rx_task, "uart1_modbus_rx_task", 1024 * 4, NULL, configMAX_PRIORITIES - 1, &TaskHandle_uart1_modbus_rx_task, 1);           
}

void end_modbus_uart_task(void)
{
    vTaskDelete(TaskHandle_uart1_modbus_rx_task);
    uart_driver_delete(MODBUS_UART_NUM);
}

_enum_fpm_modbus_write fpm_modbus_write(_enum_fpm_modbus_write _enum_mb_write)
{
    static _enum_fpm_modbus_write enum_mb_write;
    static const char delimeter[2] = " ";
    static char *token;
    static uint8_t mbrx_dt_cnt;
    static uint32_t uartinit_delay;
    static uint8_t MODBUSWRITE_dt[100];
    enum_mb_write = _enum_mb_write;
    if(enum_mb_write == MODBUSWRITE_SEND)
    {
        mbrx_dt_cnt = 0;

        token = strtok(modbus_write_str, delimeter);
        while(token != NULL)
        {
            MODBUSWRITE_dt[mbrx_dt_cnt] = (uint8_t)strtol(token, NULL, 16);
            mbrx_dt_cnt++;
            token = strtok(NULL, delimeter);
        }
        enum_mb_write = MODBUSWRITE_WRITE_SEND;
        enum_internal_modbus_operation = MODBUS_WRITE_GENERIC_WAIT;
        start_modbus_uart_task();
        uartinit_delay = xTaskGetTickCount();
    }
    else if((enum_modbus_write == MODBUSWRITE_WRITE_SEND) && (xTaskGetTickCount() - uartinit_delay > UARTINIT_DELAY))
    {
        init_modbus_rw();
        for(uint8_t i = 0; i < mbrx_dt_cnt; i++)
        {
            modbus_serial_putc(MODBUSWRITE_dt[i]);
        }
        modbus_serial_send_crc();
        enum_mb_write = MODBUSWRITE_WAIT_RESPONSE;
        modbus_get_timestamp = xTaskGetTickCount();
    }
    else if(enum_mb_write == MODBUSWRITE_WAIT_RESPONSE)
    {
        if(modbus_serial_state != MODBUS_RXCOMPLETE)
        {
            if(enum_internal_modbus_operation == MODBUS_WRITE_GENERIC_WAIT)
            {
                if(xTaskGetTickCount() - modbus_get_timestamp > modbus_timeout)
                {
                    end_modbus_uart_task();
                    enum_mb_write = MODBUSWRITE_NOT_OK;
                    modbus_error = (uint32_t)modbus_rx.error_code[0];
                    enum_internal_modbus_operation = MODBUS_ITERATE_CID;
                }
            }
        }
        else
        {
            end_modbus_uart_task();
            enum_mb_write = MODBUSWRITE_OK;
            enum_internal_modbus_operation = MODBUS_ITERATE_CID;
        }
    }
    return enum_mb_write;
}


_enum_fpm_modbus_read fpm_modbus_read_jSON(char *msg_init, char * json_string, uint16_t *json_str_len)
{
    static const modbus_operation_parameter_descriptor_t* operation_descriptor;
    static uint8_t _return = 0;
    static uint8_t raw_data_reassembly[4];
    static uint16_t i;
    static uint16_t *ptr16;
    static cJSON *wago_array;
    static cJSON *parameter_name_value_unit_obj;
    static cJSON *parameter_name_j;
    static cJSON *value_j;
    static cJSON *unit_j;
    static cJSON *sensor_json_obj;
    static char * json_string_;
    static char value_string[40];
    static char units_string[20];
    static uint32_t uartinit_delay;

    if(enum_internal_modbus_operation == MODBUS_ITERATE_CID)
    {
        if(cid == 0)
        {    
            for(i = 0; i < cid_operation_count; i++)
            {
                modbus_try_cnt[i] = 0;
                modbus_operation_result[i] = false;
                modbus_error_code[i] = 0;
            }
        }
        while(cid < cid_operation_count)
        {
            if(modbus_operation_enable[cid] == true)
            {
                operation_descriptor = &modbus_operation_parameters[cid];
                start_modbus_uart_task();
                enum_internal_modbus_operation = MODBUS_UARTINIT_DELAY;
                uartinit_delay = xTaskGetTickCount();
                modbus_try_cnt[cid]++;
                break;
            }
            else
            {
                cid++;
            }
        }
        _return = MODBUSREAD_JSON_NOT_READY;
    }
    else if((enum_internal_modbus_operation == MODBUS_UARTINIT_DELAY) && (xTaskGetTickCount() - uartinit_delay > UARTINIT_DELAY))
    {
        void* temp_data_ptr = master_get_param_data(operation_descriptor);
        assert(temp_data_ptr);
        init_modbus_rw();
        if(operation_descriptor->access == PAR_PERMS_READ)
        {
            modbus_read_holding_registers(operation_descriptor->mb_slave_addr,  operation_descriptor->mb_reg_start, operation_descriptor->param_size);
            enum_internal_modbus_operation = MODBUS_READ_WAIT;
            modbus_get_timestamp = xTaskGetTickCount();
        }
        else
        {
            enum_internal_modbus_operation = MODBUS_ITERATE_CID;
        }
        _return = MODBUSREAD_JSON_NOT_READY;
    }
    else if(enum_internal_modbus_operation == MODBUS_READ_WAIT)
    {
        if(modbus_serial_state != MODBUS_RXCOMPLETE)
        {
            if(xTaskGetTickCount() - modbus_get_timestamp > modbus_timeout)
            {
                end_modbus_uart_task();
                operation_descriptor->modbus_operation_result[cid] = false;
                enum_internal_modbus_operation = MODBUS_ITERATE_CID;
                operation_descriptor->error_code[cid] = modbus_rx.error_code[0];
                if(operation_descriptor->error_code[cid] == GATEWAY_TARGET_NO_RESPONSE)
                {
                    if(modbus_try_cnt[cid] < 5)
                    {
                        cid--;
                    }
                }
                _return = MODBUSREAD_JSON_NOT_READY;
            }
        }
        else
        {
            end_modbus_uart_task();
            operation_descriptor->modbus_operation_result[cid] = true;
            raw_data_reassembly[0] = 0;
            raw_data_reassembly[0] = 0;
            raw_data_reassembly[0] = 0;
            raw_data_reassembly[0] = 0;
            void* temp_data_ptr = master_get_param_data(operation_descriptor);
            assert(temp_data_ptr);
            if(operation_descriptor->param_type == PARAM_TYPE_U16)
            {
                raw_data_reassembly[0] = modbus_rx.data[1];
                raw_data_reassembly[1] = modbus_rx.data[0];
                *(float*)temp_data_ptr = 0;
                *(int16_t*)temp_data_ptr = *(int16_t*)raw_data_reassembly;
            }
            else if(operation_descriptor->param_type == PARAM_TYPE_HEX16)
            {
                raw_data_reassembly[0] = modbus_rx.data[1];
                raw_data_reassembly[1] = modbus_rx.data[0];
                *(float*)temp_data_ptr = 0;
                *(uint16_t*)temp_data_ptr = *(uint16_t*)raw_data_reassembly;
            }
            else if(operation_descriptor->param_type == PARAM_TYPE_HEX32)
            {
                raw_data_reassembly[0] = modbus_rx.data[3];
                raw_data_reassembly[1] = modbus_rx.data[2];
                raw_data_reassembly[2] = modbus_rx.data[1];
                raw_data_reassembly[3] = modbus_rx.data[0];
                *(uint32_t*)temp_data_ptr = *(uint32_t*)raw_data_reassembly;
            }
            else if(operation_descriptor->param_type == PARAM_TYPE_BIN32)
            {
                raw_data_reassembly[0] = modbus_rx.data[3];
                raw_data_reassembly[1] = modbus_rx.data[2];
                raw_data_reassembly[2] = modbus_rx.data[1];
                raw_data_reassembly[3] = modbus_rx.data[0];
                *(uint32_t*)temp_data_ptr = *(uint32_t*)raw_data_reassembly;
            }
            else if(operation_descriptor->param_type == PARAM_TYPE_U32)
            {
                raw_data_reassembly[0] = modbus_rx.data[3];
                raw_data_reassembly[1] = modbus_rx.data[2];
                raw_data_reassembly[2] = modbus_rx.data[1];
                raw_data_reassembly[3] = modbus_rx.data[0];
                *(int32_t*)temp_data_ptr = *(int32_t*)raw_data_reassembly;
            }
            else if(operation_descriptor->param_type == PARAM_TYPE_FLOAT)
            {
                raw_data_reassembly[0] = modbus_rx.data[3];
                raw_data_reassembly[1] = modbus_rx.data[2];
                raw_data_reassembly[2] = modbus_rx.data[1];
                raw_data_reassembly[3] = modbus_rx.data[0];
                *(float*)temp_data_ptr = 0; 
                *(float*)temp_data_ptr = *(float*)raw_data_reassembly;                
            }
            else if(operation_descriptor->param_type == PARAM_TYPE_ASCII)
            {
                raw_data_reassembly[0] = modbus_rx.data[1];
                raw_data_reassembly[1] = modbus_rx.data[0];
                *(float*)temp_data_ptr = 0;
                *(char*)temp_data_ptr = *(char*)raw_data_reassembly;
            }
            enum_internal_modbus_operation = MODBUS_ITERATE_CID;
            _return = MODBUSREAD_JSON_NOT_READY;
        }
    }
    if(enum_internal_modbus_operation == MODBUS_ITERATE_CID) 
    {
        cid++;
        if(cid < cid_operation_count)
        {
            enum_internal_modbus_operation = MODBUS_ITERATE_CID;
            _return = MODBUSREAD_JSON_UARTFREE;
        }
        else
        {   
            cid = 0;
            sensor_json_obj = cJSON_CreateObject();
            wago_array = cJSON_CreateArray();
            cJSON_AddItemToObject(sensor_json_obj, "WAGO8793040", wago_array);
            for(uint16_t cid = 0; cid < cid_operation_count; cid++)
            {    
                operation_descriptor = &modbus_operation_parameters[cid]; 
                if((operation_descriptor->access == PAR_PERMS_READ) && (operation_descriptor->modbus_operation_enable[cid] == true))
                {
                    void* temp_data_ptr = master_get_param_data(operation_descriptor);
                    parameter_name_value_unit_obj = cJSON_CreateObject();
                    cJSON_AddItemToArray(wago_array, parameter_name_value_unit_obj);
                    parameter_name_j = cJSON_CreateString(operation_descriptor->param_key);
                    cJSON_AddItemToObject(parameter_name_value_unit_obj, "parameter", parameter_name_j);
                    if((operation_descriptor->modbus_operation_result[cid] == true))
                    {   
                        if(operation_descriptor->cid == CID_R_401F_CT_ratio_2_A_Signed)
                        {
                            ptr16 = (uint16_t*)temp_data_ptr;
                            sprintf(value_string, "%u/%u", ptr16[1], ptr16[0]);
                            value_j = cJSON_CreateString(value_string);                        
                        }
                        else if(operation_descriptor->cid == CID_R_4021_Pulse_width_2_ms_Signed)
                        {
                            ptr16 = (uint16_t*)temp_data_ptr;
                            sprintf(value_string, "%u~%u", ptr16[0], ptr16[1]);
                            value_j = cJSON_CreateString(value_string);
                        }
                        else
                        {
                            if(operation_descriptor->param_type == PARAM_TYPE_U16)
                            {
                                sprintf(value_string, "%u", *(uint16_t*)temp_data_ptr);
                                value_j = cJSON_CreateString(value_string);
                            }
                            else if(operation_descriptor->param_type == PARAM_TYPE_BIN16)
                            {
                                ptr16 = (uint16_t*)temp_data_ptr;
                                sprintf(value_string, "%04X", ptr16[0]);
                                value_j = cJSON_CreateString(value_string);
                            }
                            else if(operation_descriptor->param_type == PARAM_TYPE_HEX16)
                            {
                                ptr16 = (uint16_t*)temp_data_ptr;
                                sprintf(value_string, "%04X", *ptr16);
                                value_j = cJSON_CreateString(value_string);
                            }
                            else if(operation_descriptor->param_type == PARAM_TYPE_HEX32)
                            {
                                sprintf(value_string, "%08lX", *(uint32_t*)temp_data_ptr);
                                value_j = cJSON_CreateString(value_string);
                            }
                            else if(operation_descriptor->param_type == PARAM_TYPE_U32)
                            {
                                sprintf(value_string, "%li", *(int32_t*)temp_data_ptr);
                                value_j = cJSON_CreateString(value_string);
                            }
                            else if(operation_descriptor->param_type == PARAM_TYPE_BIN32)
                            {
                                sprintf(value_string, "%08lX", *(uint32_t*)temp_data_ptr);
                                value_j = cJSON_CreateString(value_string);
                            }
                            else if(operation_descriptor->param_type == PARAM_TYPE_FLOAT)
                            {
                                static char temp_float_string[25];
                                if(cid == CID_R_4005_Protocol_version_2__Float || cid == CID_R_4007_Software_version_2__Float || cid == CID_R_4009_Hardware_version_2__Float)
                                {                
                                    static char assembly_str[10];
                                    sprintf(temp_float_string, "V%f", *(float*)temp_data_ptr);
                                    strcpy(assembly_str, &temp_float_string[3]);
                                    temp_float_string[2] = 0;
                                    strcat(temp_float_string, assembly_str);
                                    temp_float_string[4] = 0;
                                }
                                else
                                {
                                    sprintf(temp_float_string, "%0.3f", *(float*)temp_data_ptr);
                                }
                                value_j = cJSON_CreateString(temp_float_string);
                            }
                            else if(operation_descriptor->param_type == PARAM_TYPE_ASCII)
                            {
                                static char chr[2];
                                chr[1] = 0;
                                chr[0] = *(char*)temp_data_ptr;
                                value_j = cJSON_CreateString(chr);
                            }
                        }
                    }
                    else
                    {
                        sprintf(value_string, "Error(%d)", operation_descriptor->error_code[cid]);
                        value_j = cJSON_CreateString(value_string);
                    }
                    cJSON_AddItemToObject(parameter_name_value_unit_obj, "value", value_j);
                    if(strcmp(operation_descriptor->param_units, "")!= 0)
                    {
                        sprintf(units_string, "(%s)",operation_descriptor->param_units);
                        unit_j = cJSON_CreateString(units_string);
                    }
                    else
                    {
                        unit_j = cJSON_CreateString(operation_descriptor->param_units);
                    }
                    cJSON_AddItemToObject(parameter_name_value_unit_obj, "unit", unit_j);
                }
            }
            json_string_ = cJSON_Print(sensor_json_obj);

            sprintf(json_string, "%s%s", msg_init, json_string_);
            *json_str_len = strlen(json_string);
            cJSON_free(json_string_);
            cJSON_Delete(sensor_json_obj);
            enum_internal_modbus_operation = MODBUS_ITERATE_CID;
            _return = MODBUSREAD_JSON_READY;
        }
    }
    return _return;
}

void modbus_restart_cid(void)
{
    cid = 0;
}

void init_fpm_modbus(uint8_t set)
{
    static uint16_t i;
    if(set == WAGO_SET_INFO)
    {
        for(i = 0; i < CID_RW_COUNT; i++)
        {
            modbus_operation_enable[i] = false;
        }
        modbus_operation_enable[CID_R_4000_Serial_number_2__HEX] = true;
        modbus_operation_enable[CID_R_4002_Meter_code_1__HEX] = true;
        modbus_operation_enable[CID_R_4003_Modbus_ID_1__Signed] = true;
        modbus_operation_enable[CID_R_4004_Baud_rate_1__Signed] = true;
        modbus_operation_enable[CID_R_4005_Protocol_version_2__Float] = true;
        modbus_operation_enable[CID_R_4007_Software_version_2__Float] = true;
        modbus_operation_enable[CID_R_4009_Hardware_version_2__Float] = true;
        modbus_operation_enable[CID_R_400B_Meter_amps_1_A_Signed] = true;
        modbus_operation_enable[CID_R_400C_CT_ratio_1_A_HEX] = true;
        modbus_operation_enable[CID_R_400D_S0_output_rate_2_kWh_Float] = true;
        modbus_operation_enable[CID_R_400F_Combination_code_1__HEX] = true;
        modbus_operation_enable[CID_R_4010_LCD_cycle_time_1_sec_HEX] = true;
        modbus_operation_enable[CID_R_4011_Parity_setting_1__Signed] = true;
        modbus_operation_enable[CID_R_4012_Current_direction_1__ASCII] = true;
        modbus_operation_enable[CID_R_4013_L2_Current_direction_1__ASCII] = true;
        modbus_operation_enable[CID_R_4014_L3_Current_direction_1__ASCII] = true;
        modbus_operation_enable[CID_R_4016_Power_down_counter_1__Signed] = true;
        modbus_operation_enable[CID_R_4017_Present_quadrant_1__Signed] = true;
        modbus_operation_enable[CID_R_4018_L1_Quadrant_1__Signed] = true;
        modbus_operation_enable[CID_R_4019_L2_Quadrant_1__Signed] = true;
        modbus_operation_enable[CID_R_401A_L3_Quadrant_1__Signed] = true;
        modbus_operation_enable[CID_R_401B_Checksum_2__HEX] = true;
        modbus_operation_enable[CID_R_401D_Active_status_word_2__HEX] = true;
        modbus_operation_enable[CID_R_401F_CT_ratio_2_A_Signed] = true;
        modbus_operation_enable[CID_R_4021_Pulse_width_2_ms_Signed] = true;
        modbus_operation_enable[CID_R_4022_Pulse_type_setting_1_HEX] = true;
        modbus_operation_enable[CID_R_4023_Checksum_2_2__HEX] = true;
        modbus_operation_enable[CID_R_4026_Data_type_setting_1__Signed] = true;
        modbus_operation_enable[CID_R_4032_Screen_direction_1__Signed] = true;
        modbus_operation_enable[CID_R_4033_OBIS_code_1__Signed] = true;

        modbus_operation_enable[CID_R_6048_Tariff_1__Signed] = true;
    }
    else if(set == WAGO_SET_ELEC)
    {
        for(i = 0; i < CID_RW_COUNT; i++)
        {
            modbus_operation_enable[i] = true;
        }
        modbus_operation_enable[CID_R_4000_Serial_number_2__HEX] = false;
        modbus_operation_enable[CID_R_4002_Meter_code_1__HEX] = false;
        modbus_operation_enable[CID_R_4003_Modbus_ID_1__Signed] = false;
        modbus_operation_enable[CID_R_4004_Baud_rate_1__Signed] = false;
        modbus_operation_enable[CID_R_4005_Protocol_version_2__Float] = false;
        modbus_operation_enable[CID_R_4007_Software_version_2__Float] = false;
        modbus_operation_enable[CID_R_4009_Hardware_version_2__Float] = false;
        modbus_operation_enable[CID_R_400B_Meter_amps_1_A_Signed] = false;
        modbus_operation_enable[CID_R_400C_CT_ratio_1_A_HEX] = false;
        modbus_operation_enable[CID_R_400D_S0_output_rate_2_kWh_Float] = false;
        modbus_operation_enable[CID_R_400F_Combination_code_1__HEX] = false;
        modbus_operation_enable[CID_R_4010_LCD_cycle_time_1_sec_HEX] = false;
        modbus_operation_enable[CID_R_4011_Parity_setting_1__Signed] = false;
        modbus_operation_enable[CID_R_4012_Current_direction_1__ASCII] = false;
        modbus_operation_enable[CID_R_4013_L2_Current_direction_1__ASCII] = false;
        modbus_operation_enable[CID_R_4014_L3_Current_direction_1__ASCII] = false;
        modbus_operation_enable[CID_R_4016_Power_down_counter_1__Signed] = false;
        modbus_operation_enable[CID_R_4017_Present_quadrant_1__Signed] = false;
        modbus_operation_enable[CID_R_4018_L1_Quadrant_1__Signed] = false;
        modbus_operation_enable[CID_R_4019_L2_Quadrant_1__Signed] = false;
        modbus_operation_enable[CID_R_401A_L3_Quadrant_1__Signed] = false;
        modbus_operation_enable[CID_R_401B_Checksum_2__HEX] = false;
        modbus_operation_enable[CID_R_401D_Active_status_word_2__HEX] = false;
        modbus_operation_enable[CID_R_401F_CT_ratio_2_A_Signed] = false;
        modbus_operation_enable[CID_R_4021_Pulse_width_2_ms_Signed] = false;
        modbus_operation_enable[CID_R_4022_Pulse_type_setting_1_HEX] = false;
        modbus_operation_enable[CID_R_4023_Checksum_2_2__HEX] = false;
        modbus_operation_enable[CID_R_4026_Data_type_setting_1__Signed] = false;
        modbus_operation_enable[CID_R_4032_Screen_direction_1__Signed] = false;
        modbus_operation_enable[CID_R_4033_OBIS_code_1__Signed] = false;

        modbus_operation_enable[CID_R_6048_Tariff_1__Signed] = false;
    }

    modbus_restart_cid();
    enum_internal_modbus_operation = MODBUS_ITERATE_CID;
}