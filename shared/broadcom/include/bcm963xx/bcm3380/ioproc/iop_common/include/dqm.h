// dqm.h
#ifndef DQM_H
#define DQM_H

#include "io_proc.h"
#include "InOutMsgFifo.h"

// This should be in test paramater header file
#define RX_CMD_Q                0
#define TX_CMD_Q                1

// for now we are using all the shared memory
//#define DQM_TOTAL_MEM   ((SHARED_MEM_END - SHARED_MEM_BASE) >> 2) + 1
// DTP shared mem size is 32KB so for test to pass both we set to what
// UTP shared mem size, which is 16KB.
#define DQM_TOTAL_MEM   (16 * 1024)>>2
#define DQM_START_ADDR  0

// DQM Config structure
typedef struct DQMQueueConfig_S
{
    uint8   consumer;
    uint8   disableIrq;
    uint8   queue;          // which queue to configure (0 - 31)
            #define CONFIG_Q_MASK       0x1F
            #define CONIFG_CONSUMER     0x80
            #define CONFIG_IRQ_DISABLE  0x40
    uint8   tokenSize;      // in words, max=4 owrds
    uint16  qMemSize;       // in words and multiple of tokenSize
    uint16  lowWaterMark;   // in number of of tokens
    uint16  qStartAddr;     // address in Shared Memory this q started
} DQMQueueConfig_S;
#define GET_Q_FROM_Q_CONFIG_STRUCT(x, y) (x = y & CONFIG_Q_MASK)
#define IS_4KE_CONSUMER(x)               ((x & CONFIG_CONSUMER) == CONFIG_CONSUMER)
#define SET_4KE_CONSUMER(x)              (x & CONFIG_Q_MASK | CONFIG_CONSUMER)
#define SET_4KE_PRODUCER(x)              (x & CONFIG_Q_MASK)

// initialize DQM memory allocator start address
void DqmMemInit(void);

// Setting starting address and size of DQM memory space
void ConfigDqmMemSpace(uint16 startAddr, uint16 memSize);

// queue < 32, tokenSize = 1..4 (in words, single word is 32 bit)
// queueMemSize = must fit integer multiples of tokens as defined
// by Token Size, startAddr = word start address for this queue
// lowWatermark = low watermark threshold in tokens
uint8 ConfigDqmQueue (uint8 queue, uint8 tokenSize, uint16 queueMemSize, uint16 lowWatermark);

// Configure DQM queue:
// the input params pass in via DQMQueueConfig structure.  Caller must populate
// all members except qStartAddr.  This function will assign a block of shared
// memory and start address.  Caller can check qStartAddr to verify whether
// the operation is successful(there is enough memory for this request).
// qStartAddr == 0xFFFF => FAILED, could not allocate memory
// qStartAddr != 0xFFFF => PASSED.
void ConfigDqmQ( DQMQueueConfig_S * config );

#endif
