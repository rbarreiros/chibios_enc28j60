#ifndef __ENC28J60_H__
#define __ENC28J60_H__

#include <hal.h>

/*
  We assume SPI pins are setup in board.h
  remember:
  SCK -  PAL_MODE_STM32_ALTERNATE_PUSHPULL
  MISO - PAL_MODE_STM32_ALTERNATE_PUSHPULL
  MOSI - PAL_MODE_STM32_ALTERNATE_PUSHPULL
  
  These we setup:

  CS - PAL_MODE_OUTPUT_PUSHPULL
  RESET - PAL_MODE_OUTPUT_PUSHPULL
  INT - PAL_MODE_INPUT_PULLUP
 */

typedef struct
{
  ioportid_t port;
  uint8_t pin;
} EncIOPort;

typedef struct
{
  EncIOPort cs;
  EncIOPort reset;
  EncIOPort interrupt;
} EncCompPins;

typedef struct
{
  EncIOPort sck;
  EncIOPort miso;
  EncIOPort mosi;
} EncSPIPins;

typedef struct
{
  SPIDriver *drv;
  SPIConfig cfg;
  uint8_t mac[6];
  EncCompPins cmppins;
  EncSPIPins spipins;
} EncConfig;

// buffer boundaries applied to internal 8K ram
// the entire available packet buffer space is allocated

#define RXSTART_INIT        0x0000  // start of RX buffer, (must be zero, Rev. B4 Errata point 5)
#define RXSTOP_INIT         0x0BFF  // end of RX buffer, room for 2 packets

#define TXSTART_INIT        0x0C00  // start of TX buffer, room for 1 packet
#define TXSTOP_INIT         0x11FF  // end of TX buffer

#define SCRATCH_START       0x1200  // start of scratch area
#define SCRATCH_LIMIT       0x2000  // past end of area, i.e. 3 Kb
#define SCRATCH_PAGE_SHIFT  6       // addressing is in pages of 64 bytes
#define SCRATCH_PAGE_SIZE   (1 << SCRATCH_PAGE_SHIFT)
#define SCRATCH_PAGE_NUM    ((SCRATCH_LIMIT-SCRATCH_START) >> SCRATCH_PAGE_SHIFT)
#define SCRATCH_MAP_SIZE    (((SCRATCH_PAGE_NUM % 8) == 0) ? (SCRATCH_PAGE_NUM / 8) : (SCRATCH_PAGE_NUM/8+1))

// area in the enc memory that can be used via enc_malloc; by default 0 bytes; decrease SCRATCH_LIMIT in order
// to use this functionality
#define ENC_HEAP_START      SCRATCH_LIMIT
#define ENC_HEAP_END        0x2000


void encInit(EncConfig *cfg, bool buildMac);
bool encIsLinkUp(void);
void encPacketSend(uint8_t *buffer, uint16_t len);
uint16_t encPacketReceive(uint8_t *buffer, uint16_t max_len);
void encEnableBroadcast(void);
void encDisableBroadcast(void);
void encEnableMulticast(void);
void encDisableMulticast(void);
void encEnablePromiscuous(void);
void encDisablePromiscuous(void);
void encChipReset(void);
void encGetMac(uint8_t *mac);
#endif
