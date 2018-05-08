#include "enc28j60.h"

#include <hal.h>
#include <ch.h>

// debug
#include "debug.h"

// ENC28J60 Control Registers
// Control register definitions are a combination of address,
// bank number, and Ethernet/MAC/PHY indicator bits.
// - Register address        (bits 0-4)
// - Bank number        (bits 5-6)
// - MAC/PHY indicator        (bit 7)
#define ADDR_MASK        0x1F
#define BANK_MASK        0x60
#define SPRD_MASK        0x80
// All-bank registers
#define EIE              0x1B
#define EIR              0x1C
#define ESTAT            0x1D
#define ECON2            0x1E
#define ECON1            0x1F
// Bank 0 registers
#define ERDPT           (0x00|0x00)
#define EWRPT           (0x02|0x00)
#define ETXST           (0x04|0x00)
#define ETXND           (0x06|0x00)
#define ERXST           (0x08|0x00)
#define ERXND           (0x0A|0x00)
#define ERXRDPT         (0x0C|0x00)
// #define ERXWRPT         (0x0E|0x00)
#define EDMAST          (0x10|0x00)
#define EDMAND          (0x12|0x00)
// #define EDMADST         (0x14|0x00)
#define EDMACS          (0x16|0x00)
// Bank 1 registers
#define EHT0             (0x00|0x20)
#define EHT1             (0x01|0x20)
#define EHT2             (0x02|0x20)
#define EHT3             (0x03|0x20)
#define EHT4             (0x04|0x20)
#define EHT5             (0x05|0x20)
#define EHT6             (0x06|0x20)
#define EHT7             (0x07|0x20)
#define EPMM0            (0x08|0x20)
#define EPMM1            (0x09|0x20)
#define EPMM2            (0x0A|0x20)
#define EPMM3            (0x0B|0x20)
#define EPMM4            (0x0C|0x20)
#define EPMM5            (0x0D|0x20)
#define EPMM6            (0x0E|0x20)
#define EPMM7            (0x0F|0x20)
#define EPMCS           (0x10|0x20)
// #define EPMO            (0x14|0x20)
#define EWOLIE           (0x16|0x20)
#define EWOLIR           (0x17|0x20)
#define ERXFCON          (0x18|0x20)
#define EPKTCNT          (0x19|0x20)
// Bank 2 registers
#define MACON1           (0x00|0x40|0x80)
#define MACON2           (0x01|0x40|0x80)
#define MACON3           (0x02|0x40|0x80)
#define MACON4           (0x03|0x40|0x80)
#define MABBIPG          (0x04|0x40|0x80)
#define MAIPG           (0x06|0x40|0x80)
#define MACLCON1         (0x08|0x40|0x80)
#define MACLCON2         (0x09|0x40|0x80)
#define MAMXFL          (0x0A|0x40|0x80)
#define MAPHSUP          (0x0D|0x40|0x80)
#define MICON            (0x11|0x40|0x80)
#define MICMD            (0x12|0x40|0x80)
#define MIREGADR         (0x14|0x40|0x80)
#define MIWR            (0x16|0x40|0x80)
#define MIRD            (0x18|0x40|0x80)
// Bank 3 registers
#define MAADR1           (0x00|0x60|0x80)
#define MAADR0           (0x01|0x60|0x80)
#define MAADR3           (0x02|0x60|0x80)
#define MAADR2           (0x03|0x60|0x80)
#define MAADR5           (0x04|0x60|0x80)
#define MAADR4           (0x05|0x60|0x80)
#define EBSTSD           (0x06|0x60)
#define EBSTCON          (0x07|0x60)
#define EBSTCS          (0x08|0x60)
#define MISTAT           (0x0A|0x60|0x80)
#define EREVID           (0x12|0x60)
#define ECOCON           (0x15|0x60)
#define EFLOCON          (0x17|0x60)
#define EPAUS           (0x18|0x60)

// ENC28J60 ERXFCON Register Bit Definitions
#define ERXFCON_UCEN     0x80
#define ERXFCON_ANDOR    0x40
#define ERXFCON_CRCEN    0x20
#define ERXFCON_PMEN     0x10
  #define ERXFCON_MPEN     0x08
#define ERXFCON_HTEN     0x04
#define ERXFCON_MCEN     0x02
#define ERXFCON_BCEN     0x01
// ENC28J60 EIE Register Bit Definitions
  #define EIE_INTIE        0x80
#define EIE_PKTIE        0x40
#define EIE_DMAIE        0x20
#define EIE_LINKIE       0x10
#define EIE_TXIE         0x08
  #define EIE_WOLIE        0x04
#define EIE_TXERIE       0x02
#define EIE_RXERIE       0x01
// ENC28J60 EIR Register Bit Definitions
#define EIR_PKTIF        0x40
#define EIR_DMAIF        0x20
#define EIR_LINKIF       0x10
#define EIR_TXIF         0x08
#define EIR_WOLIF        0x04
#define EIR_TXERIF       0x02
#define EIR_RXERIF       0x01
// ENC28J60 ESTAT Register Bit Definitions
#define ESTAT_INT        0x80
#define ESTAT_LATECOL    0x10
#define ESTAT_RXBUSY     0x04
#define ESTAT_TXABRT     0x02
#define ESTAT_CLKRDY     0x01
// ENC28J60 ECON2 Register Bit Definitions
#define ECON2_AUTOINC    0x80
#define ECON2_PKTDEC     0x40
#define ECON2_PWRSV      0x20
#define ECON2_VRPS       0x08
// ENC28J60 ECON1 Register Bit Definitions
#define ECON1_TXRST      0x80
#define ECON1_RXRST      0x40
#define ECON1_DMAST      0x20
#define ECON1_CSUMEN     0x10
#define ECON1_TXRTS      0x08
#define ECON1_RXEN       0x04
#define ECON1_BSEL1      0x02
#define ECON1_BSEL0      0x01
// ENC28J60 MACON1 Register Bit Definitions
#define MACON1_LOOPBK    0x10
#define MACON1_TXPAUS    0x08
#define MACON1_RXPAUS    0x04
#define MACON1_PASSALL   0x02
#define MACON1_MARXEN    0x01
// ENC28J60 MACON2 Register Bit Definitions
#define MACON2_MARST     0x80
#define MACON2_RNDRST    0x40
#define MACON2_MARXRST   0x08
#define MACON2_RFUNRST   0x04
#define MACON2_MATXRST   0x02
#define MACON2_TFUNRST   0x01
// ENC28J60 MACON3 Register Bit Definitions
#define MACON3_PADCFG2   0x80
#define MACON3_PADCFG1   0x40
#define MACON3_PADCFG0   0x20
#define MACON3_TXCRCEN   0x10
#define MACON3_PHDRLEN   0x08
#define MACON3_HFRMLEN   0x04
#define MACON3_FRMLNEN   0x02
#define MACON3_FULDPX    0x01
// ENC28J60 MICMD Register Bit Definitions
#define MICMD_MIISCAN    0x02
#define MICMD_MIIRD      0x01
// ENC28J60 MISTAT Register Bit Definitions
#define MISTAT_NVALID    0x04
#define MISTAT_SCAN      0x02
#define MISTAT_BUSY      0x01

// ENC28J60 EBSTCON Register Bit Definitions
#define EBSTCON_PSV2     0x80
#define EBSTCON_PSV1     0x40
#define EBSTCON_PSV0     0x20
#define EBSTCON_PSEL     0x10
#define EBSTCON_TMSEL1   0x08
#define EBSTCON_TMSEL0   0x04
#define EBSTCON_TME      0x02
#define EBSTCON_BISTST    0x01

// PHY registers
#define PHCON1           0x00
#define PHSTAT1          0x01
#define PHHID1           0x02
#define PHHID2           0x03
#define PHCON2           0x10
#define PHSTAT2          0x11
#define PHIE             0x12
#define PHIR             0x13
#define PHLCON           0x14

// ENC28J60 PHY PHCON1 Register Bit Definitions
#define PHCON1_PRST      0x8000
#define PHCON1_PLOOPBK   0x4000
#define PHCON1_PPWRSV    0x0800
#define PHCON1_PDPXMD    0x0100
// ENC28J60 PHY PHSTAT1 Register Bit Definitions
#define PHSTAT1_PFDPX    0x1000
#define PHSTAT1_PHDPX    0x0800
#define PHSTAT1_LLSTAT   0x0004
#define PHSTAT1_JBSTAT   0x0002
// ENC28J60 PHY PHCON2 Register Bit Definitions
#define PHCON2_FRCLINK   0x4000
#define PHCON2_TXDIS     0x2000
#define PHCON2_JABBER    0x0400
#define PHCON2_HDLDIS    0x0100

// ENC28J60 Packet Control Byte Bit Definitions
#define PKTCTRL_PHUGEEN  0x08
#define PKTCTRL_PPADEN   0x04
#define PKTCTRL_PCRCEN   0x02
#define PKTCTRL_POVERRIDE 0x01

// SPI operation codes
#define ENC28J60_READ_CTRL_REG       0x00
#define ENC28J60_READ_BUF_MEM        0x3A
#define ENC28J60_WRITE_CTRL_REG      0x40
#define ENC28J60_WRITE_BUF_MEM       0x7A
#define ENC28J60_BIT_FIELD_SET       0x80
#define ENC28J60_BIT_FIELD_CLR       0xA0
#define ENC28J60_SOFT_RESET          0xFF

// max frame length which the controller will accept:
// (note: maximum ethernet frame length would be 1518)
#define MAX_FRAMELEN      1518

// Global pointer to the struct running the enc config
static EncConfig *gCfg;
static uint8_t gEncBank = 0;

static void encSetBank(uint8_t address);

static uint8_t encReadOp(uint8_t op, uint8_t address)
{
  uint8_t buffer[3] = { op | (address & ADDR_MASK), 0, 0 };
  uint8_t rx[3] = {0};
  uint8_t result = 0;

  encSetBank(address);
  
  spiSelect(gCfg->drv);
  if(address & 0x80)
  {
    spiExchange(gCfg->drv, 3, buffer, rx);
    result = rx[2];
  }
  else
  {
    spiExchange(gCfg->drv, 2, buffer, rx);
    result = rx[1];
  } 
  spiUnselect(gCfg->drv);
  
  return result;
}

static void encWriteOp(uint8_t op, uint8_t address, uint8_t data)
{
  uint8_t buffer[2] = { (op | (address & ADDR_MASK)) , data };
  
  spiSelect(gCfg->drv);
  spiSend(gCfg->drv, 2, buffer);
  spiUnselect(gCfg->drv);
}

static void encSetBank(uint8_t address)
{
  if((address & BANK_MASK) != gEncBank)
  {
    gEncBank = address & BANK_MASK;
    encWriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_BSEL1|ECON1_BSEL0);
    encWriteOp(ENC28J60_BIT_FIELD_SET, ECON1, gEncBank >> 5);
  }
}

static uint8_t encReadRegByte(uint8_t address)
{
  encSetBank(address);
  return encReadOp(ENC28J60_READ_CTRL_REG, address);
}

static uint16_t encReadReg(uint8_t address)
{
  return encReadRegByte(address) + (encReadRegByte(address + 1) << 8);
}

static void encWriteRegByte(uint8_t address, uint8_t data)
{
  encSetBank(address);
  encWriteOp(ENC28J60_WRITE_CTRL_REG, address, data);
}

static void encWriteReg(uint8_t address, uint16_t data)
{
  encWriteRegByte(address, data);
  encWriteRegByte(address + 1, data >> 8);
}

static uint16_t encReadPhyByte (uint8_t address)
{
  encWriteRegByte(MIREGADR, address);
  encWriteRegByte(MICMD, MICMD_MIIRD);
  while (encReadRegByte(MISTAT) & MISTAT_BUSY);
  encWriteRegByte(MICMD, 0x00);
  return encReadRegByte(MIRD + 1);
}

static void encWritePhy(uint8_t address, uint16_t data)
{
  while(encReadRegByte(MISTAT) & MISTAT_BUSY) continue;
  encWriteRegByte(MIREGADR, address);
  encWriteReg(MIWR, data);
  while(encReadRegByte(MISTAT) & MISTAT_BUSY) continue;
}

static void encReadBuf(uint16_t len, uint8_t *data)
{
  if(len == 0) return;
  uint8_t tx[1] = { ENC28J60_READ_BUF_MEM };

  spiSelect(gCfg->drv);
  spiSend(gCfg->drv, 1, tx);
  spiReceive(gCfg->drv, len, data);
  spiUnselect(gCfg->drv);
}

static void encWriteBuf(uint16_t len, uint8_t *data)
{
  if(len == 0) return;
  uint8_t tx[1] = { ENC28J60_WRITE_BUF_MEM };
  
  spiSelect(gCfg->drv);
  spiSend(gCfg->drv, 1, tx);
  spiSend(gCfg->drv, len, data);
  spiUnselect(gCfg->drv);
}

static uint8_t encInitialize(void)
{
  dbg(".. Chip Reset");
  encChipReset();

  uint8_t r = encReadRegByte(EREVID);
  dbgf(".. Rev: %d\r\n", ++r);
  
  // Soft reset
  dbg(".. Soft Reset");
  encWriteOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
  chThdSleepMilliseconds(2);
  while (!encReadOp(ENC28J60_READ_CTRL_REG, ESTAT) & ESTAT_CLKRDY);

  dbg(".. RX Pointers");
  // RX pointers
  encWriteReg(ERXST, RXSTART_INIT);
  encWriteReg(ERXRDPT, RXSTART_INIT);
  encWriteReg(ERXND, RXSTOP_INIT);

  // TX pointers
  encWriteReg(ETXST, TXSTART_INIT);
  encWriteReg(ETXND, TXSTOP_INIT);

  // RX Filters
  /**
   * ERXFCON_UCEN  - (Unicast filter) 
   *                 Packets not having a destination address matching 
   *                 the local MAC address will be discarded
   *
   * ERXFCON_CRCEN - (Post-Filter CRC Check Enable bit)  
   *                 All packets with an invalid CRC will be discarded
   *
   * ERXFCON_PMEN  - (Pattern Match Filter Enable bit) 
   *                 Packets must meet the Pattern Match criteria or 
   *                 they will be discarded
   *
   * ERXFCON_BCEN  - (Broadcast Filter Enable bit) 
   *                 Packets must have a destination address of 
   *                 FF-FF-FF-FF-FF-FF or they will be discarded
   *
   * ERXFCON_MCEN  - (Multicast Filter Enable bit) 
   *                 Packets must have the Least Significant bit set 
   *                 in the destination address or they will be discarded
   */
  
  encWriteRegByte(ERXFCON,
		  ERXFCON_UCEN | 
		  ERXFCON_CRCEN|
		  ERXFCON_PMEN |
		  ERXFCON_BCEN |
		  ERXFCON_MCEN);

  // No need to pattern match filter packets
  //encWriteReg(EPMM0, 0x303f);
  //encWriteReg(EPMCS, 0xf7f9);

  // MAC Control registers
  encWriteRegByte(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
  encWriteRegByte(MACON2, 0x00);

  encWriteOp(ENC28J60_BIT_FIELD_SET, MACON3,
	     MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN|MACON3_FULDPX);

  // Full duplex
  encWritePhy(PHCON1, PHCON1_PDPXMD);

  encWriteReg(MAIPG, 0x0C12);
  encWriteRegByte(MABBIPG, 0x12);

  // Max frame len
  encWriteReg(MAMXFL, MAX_FRAMELEN);

  // MAC Address
  encWriteRegByte(MAADR5, gCfg->mac[0]);
  encWriteRegByte(MAADR4, gCfg->mac[1]);
  encWriteRegByte(MAADR3, gCfg->mac[2]);
  encWriteRegByte(MAADR2, gCfg->mac[3]);
  encWriteRegByte(MAADR1, gCfg->mac[4]);
  encWriteRegByte(MAADR0, gCfg->mac[5]);

  // Disable loopback
  encWritePhy(PHCON2, PHCON2_HDLDIS);
  
  encSetBank(ECON1);
  encWriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE);
  encWriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);

  uint8_t rev = encReadRegByte(EREVID);
  // microchip forgot to step the number on the silcon when they
  // released the revision B7. 6 is now rev B7. We still have
  // to see what they do when they release B8. At the moment
  // there is no B8 out yet
  if (rev > 5) ++rev;
  return rev;
}

void encSpiInit(void)
{
  // This should be already setup in board.h, but lets do it anyway here !
  palSetPadMode(gCfg->spipins.sck.port, gCfg->spipins.sck.pin, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  palSetPadMode(gCfg->spipins.miso.port, gCfg->spipins.miso.pin, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  palSetPadMode(gCfg->spipins.mosi.port, gCfg->spipins.mosi.pin, PAL_MODE_STM32_ALTERNATE_PUSHPULL);

  // This are the complementary pins, same as above, should be setup in board.h !
  palSetPadMode(gCfg->cmppins.cs.port, gCfg->cmppins.cs.pin, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(gCfg->cmppins.reset.port, gCfg->cmppins.reset.pin, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(gCfg->cmppins.interrupt.port, gCfg->cmppins.interrupt.pin, PAL_MODE_INPUT_PULLUP);

  palSetPad(gCfg->cmppins.reset.port, gCfg->cmppins.reset.pin); // Reset High (No reset)
  palSetPad(gCfg->cmppins.cs.port, gCfg->cmppins.cs.pin); // SS High (Disabled)
  
  spiStart(gCfg->drv, &(gCfg->cfg));
}

void encBuildMac(void)
{
#define STM32_UUID ((uint32_t*)0x1ffff7e8)

  struct u_id
  {
    uint32_t off0;
    uint32_t off1;
    uint32_t off2;
  };

  struct u_id id;
  id.off0 = STM32_UUID[0];
  id.off1 = STM32_UUID[1];
  id.off2 = STM32_UUID[2];
  
  // Private/Local MAC has the second-least-significant
  // bit of the most significant byte set
  // x2 x6 xA xE : xx : xx : xx : xx : xx
  
  gCfg->mac[0] = (uint8_t)((id.off2 & 0xff) | ( 0x02 & 0x0f));
  gCfg->mac[1] = (uint8_t)((id.off2 >> 8) & 0xff);
  gCfg->mac[2] = (uint8_t)((id.off2 >> 16) & 0xff);
  gCfg->mac[3] = (uint8_t)(id.off1 & 0xff);
  gCfg->mac[4] = (uint8_t)((id.off1 >> 8) & 0xff);
  gCfg->mac[5] = (uint8_t)(id.off0 & 0xff);
}

// Public API

void encInit(EncConfig *cfg, bool buildMac)
{
  gCfg = cfg;

  if(buildMac)
  {
    dbg(":: Building MAC");
    encBuildMac();
  }
  
  dbg(":: Initializing SPI");
  encSpiInit();

  dbg(":: Initializing ENC");
  encInitialize();
}

bool encIsLinkUp(void)
{
  return (encReadPhyByte(PHSTAT2) >> 2) & 1;
}

struct transmit_status_vector
{
  uint8_t bytes[7];
};

void encPacketSend(uint8_t *buffer, uint16_t len)
{
  uint8_t retry = 0;
  
  while(1)
  {
    // latest errata sheet: DS80349C
    // always reset transmit logic (Errata Issue 12)
    // the Microchip TCP/IP stack implementation used to first check
    // whether TXERIF is set and only then reset the transmit logic
    // but this has been changed in later versions; possibly they
    // have a reason for this; they don't mention this in the errata
    // sheet
    encWriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
    encWriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
    encWriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXERIF|EIR_TXIF);

    if(retry == 0)
    {
      encWriteReg(EWRPT, TXSTART_INIT);
      encWriteReg(ETXND, TXSTART_INIT + len);
      encWriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);
      encWriteBuf(len, buffer);
    }
    
    encWriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);

    // wait until transmission has finished; referrring to the data sheet and
    // to the errata (Errata Issue 13; Example 1) you only need to wait until either
    // TXIF or TXERIF gets set; however this leads to hangs; apparently Microchip
    // realized this and in later implementations of their tcp/ip stack they introduced
    // a counter to avoid hangs; of course they didn't update the errata sheet
    uint16_t count = 0;
    while ((encReadRegByte(EIR) & (EIR_TXIF | EIR_TXERIF)) == 0 && ++count < 1000U);

    if (!(encReadRegByte(EIR) & EIR_TXERIF) && count < 1000U)
      break;

    // cancel previous transmission if stuck
    encWriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRTS);

    // Check whether the chip thinks that a late collision ocurred; the chip
    // may be wrong (Errata Issue 13); therefore we retry. We could check
    // LATECOL in the ESTAT register in order to find out whether the chip
    // thinks a late collision ocurred but (Errata Issue 15) tells us that
    // this is not working. Therefore we check TSV
    struct transmit_status_vector tsv;
    uint16_t etxnd = encReadReg(ETXND);
    encWriteReg(ERDPT, etxnd + 1);
    encReadBuf(sizeof(struct transmit_status_vector), (uint8_t*) &tsv);
    // LATECOL is bit number 29 in TSV (starting from 0)

    if (!((encReadRegByte(EIR) & EIR_TXERIF) && (tsv.bytes[3] & 1 << 5) /*tsv.transmitLateCollision*/) || retry > 16U)
    {
      // there was some error but no LATECOL so we do not repeat
      break;
    }

    retry++;
  }
}

uint16_t encPacketReceive(uint8_t *buffer, uint16_t max_len)
{
  static uint16_t gNextPacketPtr = RXSTART_INIT;
  static uint8_t unreleasedPacket = 0;
  uint16_t len = 0;

  if(unreleasedPacket)
  {
    if(gNextPacketPtr == 0)
      encWriteReg(ERXRDPT, RXSTOP_INIT);
    else
      encWriteReg(ERXRDPT, gNextPacketPtr - 1);

    unreleasedPacket = 0;
  }

  if(encReadRegByte(EPKTCNT) > 0)
  {
    encWriteReg(ERDPT, gNextPacketPtr);

    struct
    {
      uint16_t nextPacket;
      uint16_t byteCount;
      uint16_t status;
    } header;

    encReadBuf(sizeof(header), (uint8_t*)&header);

    gNextPacketPtr = header.nextPacket;
    len = header.byteCount - 4; // remove CRC count

    /*
    if(len > BUFFERSIZE - 1)
      len = BUFFERSIZE - 1;
    */
    // If packet length bigger than our buffer
    // just discard the packet, we must ensure
    // our buffer handles packets of all sizes
    // if it doesn't, we don't need it!
    if(len > max_len)
      return 0;

    if((header.status & 0x80) == 0)
      len = 0;
    else
      encReadBuf(len, buffer);

    buffer[len] = 0;
    unreleasedPacket = 1;

    encWriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
  }
  
  return len;
}


void encEnableBroadcast(void)
{
  encWriteRegByte(ERXFCON, encReadRegByte(ERXFCON) | ERXFCON_BCEN);
}

void encDisableBroadcast(void)
{
  encWriteRegByte(ERXFCON, encReadRegByte(ERXFCON) & ~ERXFCON_BCEN);
}

void encEnableMulticast(void)
{
  encWriteRegByte(ERXFCON, encReadRegByte(ERXFCON) | ERXFCON_MCEN);
}

void encDisableMulticast(void)
{
  encWriteRegByte(ERXFCON, encReadRegByte(ERXFCON) & ~ERXFCON_MCEN);
}

void encEnablePromiscuous(void)
{
  encWriteRegByte(ERXFCON, encReadRegByte(ERXFCON) & ERXFCON_CRCEN);
}

void encDisablePromiscuous(void)
{
  encWriteRegByte(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN|ERXFCON_BCEN);
}

void encChipReset(void)
{
  palClearPad(gCfg->cmppins.reset.port, gCfg->cmppins.reset.pin);
  chThdSleepMilliseconds(500);
  palSetPad(gCfg->cmppins.reset.port, gCfg->cmppins.reset.pin);
}

void encGetMac(uint8_t *mac)
{
  int i;
  for(i = 0; i < 6; i++)
    mac[i] = gCfg->mac[i];
}
