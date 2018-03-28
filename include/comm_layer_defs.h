#ifndef CL_STATUS_DEF_H
#define CL_STATUS_DEF_H

/*
 * Status
 */
typedef enum
{
  CL_OK                   = 0x00U,
  CL_ERROR                = 0x01U,
  CL_BUSY                 = 0x02U,
  CL_RECEIVED_INSTRUCTION = 0x03U
} CommLayer_StatusTypeDef;

typedef enum
{
  CL_TX_ERROR             = 0x01U,
  CL_RX_ERROR             = 0x02U,
  CL_CHECKSUM_ERROR       = 0x03U,
  CL_DEV_ERROR            = 0x04U,
  CL_PARAM_ERROR          = 0x05U,
  CL_RX_DATA_LENGTH_ERROR = 0x06U,
  CL_TASK_QUEUED          = 0x07U
} CommLayer_ErrorTypeDef;

class cl_error : public std::runtime_error
{
public:
  explicit cl_error(std::string error_msg) : std::runtime_error(error_msg) {};
};

class cl_tx_error : public cl_error
{
public:
  explicit cl_tx_error(std::string error_msg) : cl_error(error_msg) {};
};

class cl_rx_error : public cl_error
{
public:
  explicit cl_rx_error(std::string error_msg) : cl_error(error_msg) {};
};

class cl_checksum_error : public cl_error
{
public:
  explicit cl_checksum_error(std::string error_msg) : cl_error(error_msg) {};
};

class cl_device_error : public cl_error
{
public:
  explicit cl_device_error(std::string error_msg) : cl_error(error_msg) {};
};

/*
 * Dynamixel
 */
#pragma pack(1)
typedef struct
{
  uint8_t instruction;
  uint16_t param;
} DynamixelMessageFields;
#pragma pack()

/*
 * General
 */
/*
 * Instuction Types:
 * - Status: 0-15
 * - Dynamixel: 16-31
 * - CAN: 32-47
 */
typedef enum
{
  DYN_SET_POSITION             = 0x16U,
  DYN_GET_POSITION_INSTRUCTION = 0x17U, // Sending response
  DYN_GET_POSITION_RESPONSE    = 0x18U, // Receiving command
  DYN_SET_VELOCITY             = 0x19U,
  DYN_GET_VELOCITY             = 0x20U,
  DYN_SET_DELAY                = 0x21U,
  DYN_SET_COMPLIANCE_MARGIN    = 0x22U,
  DYN_SET_COMPLIANCE_SLOPE     = 0x23U,
  DYN_SET_BAUD_RATE            = 0x24U,
  DYN_SET_POLLING_DT           = 0x25U,
  DYN_ADJUST_SERVO             = 0x26U
} CommLayer_InstructionTypeDef;

#pragma pack(1)
typedef struct
{
  uint8_t instruction;
  uint16_t data;
  uint8_t checksum;
} UCLMessage32Field;
#pragma  pack()

#pragma pack(1)
typedef struct
{
  uint8_t instruction;
  int16_t data;
  uint8_t checksum;
} CLMessage32Field;
#pragma  pack()


typedef union
{
  UCLMessage32Field      ucl;
  CLMessage32Field       cl;
  DynamixelMessageFields dyn;
  uint8_t                data8[4];
  uint32_t               data32;
} CLMessage32;

#endif //CL_STATUS_DEF_H