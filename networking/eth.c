#include "eth.h"
#include "em_gpio.h"
#include "em_device.h"
#include "em_cmu.h"

// LWIP Includes
#include "lwip/opt.h"
#include "lwip/timeouts.h"
#include "lwip/tcpip.h"

// MICRIUM Includes
#include "os.h"
#include <rtos_description.h>
#include "cpu_cache.h"

#define RX_DESCRIPTOR_QUEUE_SIZE 5
#define TX_DESCRIPTOR_QUEUE_SIZE 12

// Maximum size of an ethernet frame (1518) and then align to 32 bits (1536)
// The DMA engine exepcts the address words to point to 32 bit boundaries.
#define NET_IF_ETHER_FRAME_MAX_SIZE 1536

// Function to setup the GPIO pins for ETH
static void ETH_GPIOInitalize(void);

// Function to read the current link state from the phy
static uint8_t ETH_GetPhyLinkState(void);

// Function to get LWIP running
static void LWIP_Initalize(void);

// Create the received network data buffers
static uint8_t received_network_data_buffer[RX_DESCRIPTOR_QUEUE_SIZE][NET_IF_ETHER_FRAME_MAX_SIZE];

// Define the DMA descriptor type
typedef struct dma_descriptor {
  uint32_t addr;
  uint32_t status;
} dma_descriptor_t;

// Define the DMA descriptor table
// The queues could be dynamically allocated using Mem_SegAllocExt but
// this is beyond the scope of this demo
typedef struct dma_descriptor_list {
  dma_descriptor_t ReceiveBufferQueues[RX_DESCRIPTOR_QUEUE_SIZE];
  dma_descriptor_t TransmitBufferQueues[TX_DESCRIPTOR_QUEUE_SIZE];
  dma_descriptor_t *ReceiveBufferQueueStart;
  dma_descriptor_t *ReceiveBufferQueueCur;
  dma_descriptor_t *ReceiveBufferQueueEnd;
  dma_descriptor_t *TransmitBufferQueueStart;
  dma_descriptor_t *TransmitBufferQueueCur;
  dma_descriptor_t *TransmitBufferQueueEnd;
  dma_descriptor_t *TransmitBufferQueueAcked;
} dma_descriptor_list_t;

// Create the DMA descriptor list by declaring it here
// Again this could be done dynamically if required.
static dma_descriptor_list_t dma_descriptor_list;

// Define a custom PBUF struct
typedef struct
{
  struct pbuf_custom pbuf_custom;
  dma_descriptor_t dma_descriptor;
} RxBuff_t;

// Declare the LWIP Memory pool using our custom PBUF
#define ETH_RX_BUFFER_CNT             12U // Holds up to 12 buffers
LWIP_MEMPOOL_DECLARE(RX_POOL, ETH_RX_BUFFER_CNT, sizeof(RxBuff_t), "Zero-copy RX PBUF pool");


// Called to initialize everything required for Ethernet
void Eth_Initalize()
{
  // Enable the High Frequency Periphrial Clocks
  CMU_ClockEnable(cmuClock_HFPER, true);
  // Enable the GPIO clock
  CMU_ClockEnable(cmuClock_GPIO, true);
  // Enable the ETH clock
  CMU_ClockEnable(cmuClock_ETH, true);

  // Set up the GPIO pins
  ETH_GPIOInitalize();

  // See figure 40.5 in the EFM32GG11 reference manual
  // Send HFXO to PHY clock pin via PD10
  GPIO_PinModeSet(gpioPortD,  10, gpioModePushPull, 0);
  // Set CLKOUT2 source to the HFXO
  CMU->CTRL |= CMU_CTRL_CLKOUTSEL2_HFXO;
  // Route CLKOUT2 to Location 5 (PD10)
  CMU->ROUTELOC0 = CMU_ROUTELOC0_CLKOUT2LOC_LOC5;
  // Enable the route
  CMU->ROUTEPEN |= CMU_ROUTEPEN_CLKOUT2PEN;
  // Enable the ETH clocks and set RMII mode
  ETH->CTRL = ETH_CTRL_GBLCLKEN | ETH_CTRL_MIISEL_RMII;

  // Initialize the receive descriptor pointers
  dma_descriptor_list.ReceiveBufferQueueStart = &dma_descriptor_list.ReceiveBufferQueues[0];
  dma_descriptor_list.ReceiveBufferQueueCur = &dma_descriptor_list.ReceiveBufferQueues[0];
  dma_descriptor_list.ReceiveBufferQueueEnd = &dma_descriptor_list.ReceiveBufferQueues[RX_DESCRIPTOR_QUEUE_SIZE - 1];

  for(int i = 0; i < RX_DESCRIPTOR_QUEUE_SIZE; i++){
      // Set the address in the descriptor list to the address of the buffer
      dma_descriptor_list.ReceiveBufferQueues[i].addr = (uint32_t) &received_network_data_buffer[i][0];
      // Clear the status bit
      dma_descriptor_list.ReceiveBufferQueues[i].addr &= ~0x01;
  }

  // Set the WRAP bit on the last descriptor
  dma_descriptor_list.ReceiveBufferQueues[RX_DESCRIPTOR_QUEUE_SIZE - 1].addr |= 0x02;

  // Invalidate DCACHE around network buffer
  CPU_DCACHE_RANGE_FLUSH(&received_network_data_buffer, NET_IF_ETHER_FRAME_MAX_SIZE * RX_DESCRIPTOR_QUEUE_SIZE)

  // Initialize the transmit descriptor pointers
  dma_descriptor_list.TransmitBufferQueueStart = &dma_descriptor_list.TransmitBufferQueues[0];
  dma_descriptor_list.TransmitBufferQueueCur = &dma_descriptor_list.TransmitBufferQueues[0];
  dma_descriptor_list.TransmitBufferQueueAcked = &dma_descriptor_list.TransmitBufferQueues[0];
  dma_descriptor_list.TransmitBufferQueueEnd = &dma_descriptor_list.TransmitBufferQueues[TX_DESCRIPTOR_QUEUE_SIZE - 1];

  for(int i = 0; i < TX_DESCRIPTOR_QUEUE_SIZE; i++){
      // Set the used bit to 1
      /*
       * must be zero for the ETH modules to read data to the transmit buffer. The ETH module sets this to
       * one for the first buffer of a frame once it has been successfully transmitted. Software must clear this bit
       * before the buffer can be used again.
       *
       */

      dma_descriptor_list.TransmitBufferQueues[i].status = 0x80000000u;
  }

  // Set the WRAP bit on the last descriptor
  dma_descriptor_list.TransmitBufferQueues[TX_DESCRIPTOR_QUEUE_SIZE - 1].status |= 0x40000000u;

  // Specify a MAC address. This would usually be read from NVM but
  // that is out of scope of this demo.
  uint8_t mac_address[6] = {0xE9, 0x2C, 0x31, 0x0A, 0x89, 0x9f};

  // Set the MAC address bottom bytes
  ETH->SPECADDR1BOTTOM = mac_address[0] << 0 | mac_address[1] << 8 |
      mac_address[2] << 16 | mac_address[3] << 24;

  // Set the MAC address top byes
  ETH->SPECADDR1TOP = mac_address[4] << 0 | mac_address[5] << 8;

  // Set the phy wake-up time, for 100Base-TX this is typically less 32 us
  ETH->SYSWAKETIME = 100; // 100 = 32 us (See Reference Manual 40.5.20 ETH_SYSWAKETIME - System wake time)

  // Enable interrupts for RX and TX complete
  ETH->IENS |= ETH_IENS_RXCMPLT | ETH_IENS_TXCMPLT;

  // Remove frame check sequence, enable unicast and multicast hashing
  ETH->NETWORKCFG |= ETH_NETWORKCFG_FCSREMOVE |
      ETH_NETWORKCFG_UNICASTHASHEN |
      ETH_NETWORKCFG_MULTICASTHASHEN;

  // Set Full duplex 100Base_TX mode
  ETH->NETWORKCFG |= ETH_NETWORKCFG_FULLDUPLEX | ETH_NETWORKCFG_SPEED;

  // Set up the ETH DMA with the RX description queue start address
  ETH->RXQPTR = (uint32_t) dma_descriptor_list.ReceiveBufferQueueStart;

  // Set up the ETH DMA with the TX description queue start address
  ETH->TXQPTR = (uint32_t) dma_descriptor_list.TransmitBufferQueueStart;

  // Enable transmitter and receiver
  ETH->NETWORKCTRL |= ETH_NETWORKCTRL_ENBRX |
    ETH_NETWORKCTRL_ENBTX |
    ETH_NETWORKCTRL_MANPORTEN;

  // Wait for the phy to get a network link
  while(ETH_GetPhyLinkState() == 0)
  {

  }




}

void ETH_GPIOInitalize(void){
  RTOS_ERR err;
  // Setup the RMII pins
  GPIO_PinModeSet(gpioPortD, 11, gpioModeInput, 0);             // CRS_DV
  GPIO_PinModeSet(gpioPortF, 7, gpioModePushPull, 0);           // TXD0
  GPIO_PinModeSet(gpioPortF, 6, gpioModePushPull, 0);           // TXD1
  GPIO_PinModeSet(gpioPortF, 8, gpioModePushPull, 0);           // TX_EN
  GPIO_PinModeSet(gpioPortD, 9, gpioModeInput, 0);              // RXD0
  GPIO_PinModeSet(gpioPortF, 9, gpioModeInput, 0);              // RXD1
  GPIO_PinModeSet(gpioPortD, 12, gpioModeInput, 0);             // RX_ER

  // Setup route locations and enable pins
  ETH->ROUTELOC1 = (1 << _ETH_ROUTELOC1_RMIILOC_SHIFT)
                   | (1 << _ETH_ROUTELOC1_MDIOLOC_SHIFT);
  ETH->ROUTEPEN = ETH_ROUTEPEN_RMIIPEN | ETH_ROUTEPEN_MDIOPEN;
  ETH->ROUTEPEN = ETH_ROUTEPEN_RMIIPEN | ETH_ROUTEPEN_MDIOPEN;

  // Setup the MDIO pins
    GPIO_PinModeSet(gpioPortD, 13, gpioModePushPull, 0);          // MDIO
    GPIO_PinModeSet(gpioPortD, 14, gpioModePushPull, 0);          // MDC

  // Enable the PHY on the STK
    GPIO_PinModeSet(gpioPortI, 10, gpioModePushPull, 1);
    GPIO_PinModeSet(gpioPortH, 7, gpioModePushPull, 1);

    /* PHY address detection is being done early in the initialization sequence
     * so we must wait for the PHY to be ready to answer to MDIO requests. */
    OSTimeDly(30, OS_OPT_TIME_DLY, &err);
}

uint8_t ETH_GetPhyLinkState(void){
  // This value is built using section 4.5.13 of the EFM32GG11 reference manual
  // It sets up a read from register 0x01 of the phy which is the basic status
  // register.
  ETH->PHYMNGMNT = 0x60060000;
  // Wait for the MDIO operation to complete by watching the flag
  while((ETH->NETWORKSTATUS & ETH_NETWORKSTATUS_MANDONE) == 0)
  {
      __asm("nop");
  }
  // Extract the link status value
  uint32_t value = (ETH->PHYMNGMNT & _ETH_PHYMNGMNT_PHYRWDATA_MASK) & 0x04;
  return value;
}

void ETH_IRQHandler(void)
{

}

void ETH_IRQEnable(void)
{
  NVIC_EnableIRQ(ETH_IRQn);
}

void ETH_IRQDisable(void)
{
  NVIC_DisableIRQ(ETH_IRQn);
}
