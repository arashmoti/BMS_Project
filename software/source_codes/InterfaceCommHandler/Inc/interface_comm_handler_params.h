#ifndef __INTERFACE_COMM_HANDLER_PARAMS__H_
#define __INTERFACE_COMM_HANDLER_PARAMS__H_

#include <cstdint>


                        






namespace SerialArgs {

constexpr uint8_t  INTERFACE_COMM_BUFFER_SIZE   = 64;

constexpr uint16_t  INTERFACE_COMM_BYTE_ESC   = 0x7D;
constexpr uint16_t INTERFACE_COMM_BYTE_START  = 0x7E;
constexpr uint16_t INTERFACE_COMM_BYTE_STATUS = 0xF8;

constexpr uint8_t kRxSignal                   = 0x1;


typedef enum _enInterfaceCommState {
  INTERFACE_COMM_STATE_IDLE                              = 0x8A,
  INTERFACE_COMM_STATE_RECEIVING                         = 0x8B,
  INTERFACE_COMM_STATE_ESCAPE                            = 0x8C
}enInterfaceCommState;


#pragma pack(push)
#pragma pack(1) 
                
typedef struct _stInterfaceCommParams {

  volatile uint8_t u8CurByte;
  volatile enInterfaceCommState curState;
  volatile uint8_t u8Index;
  volatile uint8_t u8TxIndex;
  volatile uint8_t u8PackageReady        : 1;
  volatile uint8_t u8AckReceived         : 1;
  volatile uint8_t u8NackReceived         : 1;
  volatile uint8_t u8StatusRespReceived  : 1;

  union {
    volatile uint16_t u16HasError;
    struct {
      volatile uint8_t u8NoiseError          : 1; // LSB
      volatile uint8_t u8FrameError          : 1;
      volatile uint8_t u8ParityError         : 1;
      volatile uint8_t u8OverrunError        : 1;
      volatile uint8_t u8CrcError            : 1;
      volatile uint8_t u8EscapeError         : 1;
      volatile uint8_t u8MaxSizeOverflow     : 1;
      volatile uint8_t u8FinishedWithEsc     : 1;
      volatile uint8_t u8FrameStartError     : 1;
      volatile uint8_t u8PayloadSizeError    : 1;
      uint8_t : 1;
      uint8_t : 1;
      uint8_t : 1;
      uint8_t : 1;
      uint8_t : 1;
      uint8_t : 1; // MSB

    } stInterfaceCommErrors;
  };

  union {
    volatile uint8_t arru8Bytes[INTERFACE_COMM_BUFFER_SIZE];
    struct {
      volatile uint8_t u8StartByte;
      volatile uint8_t u8PayloadLength;
      union {
        volatile uint8_t arru8Payload[INTERFACE_COMM_BUFFER_SIZE - 4];
        struct {
          volatile uint8_t u8MessageType : 8;
          volatile uint8_t arru8Parameters[INTERFACE_COMM_BUFFER_SIZE - 5];
        };
      };
    };
  } unInterfaceCommRX;

  union {
    volatile uint8_t arru8Bytes[INTERFACE_COMM_BUFFER_SIZE];
    struct {
      volatile uint8_t u8StartByte;
      volatile uint8_t u8PayloadLength;
      union {
        volatile uint8_t arru8Payload[INTERFACE_COMM_BUFFER_SIZE - 4];
        struct {
          volatile uint8_t u8MessageType : 8;
          volatile uint8_t arru8Parameters[INTERFACE_COMM_BUFFER_SIZE - 5];
        };
      };
    };
  } unInterfaceCommTX;

} stInterfaceCommParams;

// restore original alignment rules from stack
#pragma pack(pop)

}

#endif // __INTERFACE_COMM_HANDLER_PARAMS__H_