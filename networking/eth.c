#include "eth.h"
#include "em_gpio.h"
#include "em_device.h"
#include "em_cmu.h"
#include "sl_assert.h"
#include <string.h>
#include "app_log.h"
#include <stdio.h>
// LWIP Includes
#include "lwip/opt.h"
#include "lwip/timeouts.h"
#include "lwip/tcpip.h"
#include "lwip/etharp.h"
#include "lwip/udp.h"

// MICRIUM Includes
#include "os.h"
#include <rtos_description.h>


#define RX_DESCRIPTOR_QUEUE_SIZE      12
#define TX_DESCRIPTOR_QUEUE_SIZE      12
#define ETH_RX_BUFFER_CNT             12U
#define THREADING_CONFIG_RX_TASK_SIZE 600

// Maximum size of an ethernet frame (1518) and then align to 32 bits (1536)
// The DMA engine exepcts the address words to point to 32 bit boundaries.
#define NET_IF_ETHER_FRAME_MAX_SIZE 1536

// Function to setup the GPIO pins for ETH
static void ETH_GPIOInitalize(void);

// Function to enable ETH interrupts
static void ETH_IRQEnable(void);

// Function to read the current link state from the phy
static uint8_t ETH_GetPhyLinkState(void);

// Function to reset the phy over MDIO
static void ETH_ResetPhy(void);

// Function to get LWIP running
static void LWIP_Initalize(void);

// Function used to free allocations of our custom pbuf
static void ETH_PbufFree(struct pbuf *p);

// Function to send frames via ETH
err_t ETH_Output(struct netif *netif, struct pbuf *p);

// Function to recieve frames from ETH
static void ETH_RXHandler(void* param);

// Function used a call back during netif creation
err_t Eth_InitalizeInternetInterface(struct netif *netif);

// The LWIP netif used for access to the internet
struct netif internet_netif;

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
} dma_descriptor_list_t;

// Define a custom PBUF struct
typedef struct
{
  struct pbuf_custom pbuf_custom;
  uint32_t dma_descriptor_index;
} RxBuff_t;

// Create the DMA descriptor list by declaring it here
// Again this could be done dynamically if required.
static dma_descriptor_list_t dma_descriptor_list;

// Declare the LWIP Memory pool using our custom PBUF
LWIP_MEMPOOL_DECLARE(RX_POOL, ETH_RX_BUFFER_CNT, sizeof(RxBuff_t), "Zero-copy RX PBUF pool");

// Semaphore used to signal RX thread that a packet is available
static OS_SEM rx_semaphore;

// Specify a MAC address. This would usually be read from NVM but
// that is out of scope of this demo.
static uint8_t mac_address[6] = {0x20, 0x43, 0x65, 0x87, 0xA9, 0xCB};

// Used to hold the stack for the RX task
static CPU_STK rx_task_stk[THREADING_CONFIG_RX_TASK_SIZE];

// Used to hold the control block for the RX task
static OS_TCB  rx_task_tcb;

// Used to keep track of which descriptor the TX dma is pointing at
uint32_t dma_idx = 0;

// Called to initialize everything required for Ethernet
void Eth_Initalize()
{
  RTOS_ERR err;

  // Initialize the RX POOL
  LWIP_MEMPOOL_INIT(RX_POOL);

  // Create the semaphore that will be used to signal that a frame was received to the RX thread
  OSSemCreate(&rx_semaphore, "ETH RX Sem", 0, &err);

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

  for(int i = 0; i < RX_DESCRIPTOR_QUEUE_SIZE; i++){
      // Set the address in the descriptor list to the address of the buffer
      dma_descriptor_list.ReceiveBufferQueues[i].addr = (uint32_t) &received_network_data_buffer[i][0];
      // Clear the status bit
      dma_descriptor_list.ReceiveBufferQueues[i].addr &= ~0x01;
  }

  // Set the WRAP bit on the last descriptor
  dma_descriptor_list.ReceiveBufferQueues[RX_DESCRIPTOR_QUEUE_SIZE - 1].addr |= 0x02;

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

  // Set the MAC address bottom bytes
  ETH->SPECADDR1BOTTOM = mac_address[0] << 0 | mac_address[1] << 8 |
      mac_address[2] << 16 | mac_address[3] << 24;

  // Set the MAC address top byes
  ETH->SPECADDR1TOP = mac_address[4] << 0 | mac_address[5] << 8;

  // Set the phy wake-up time, for 100Base-TX this is typically less 32 us
  ETH->SYSWAKETIME = 100; // 100 = 32 us (See Reference Manual 40.5.20 ETH_SYSWAKETIME - System wake time)

  // Enable interrupts for RX complete
  ETH->IENS |= ETH_IENS_RXCMPLT;

  // Remove frame check sequence, enable unicast and multicast hashing
  ETH->NETWORKCFG |= ETH_NETWORKCFG_FCSREMOVE |
      ETH_NETWORKCFG_UNICASTHASHEN |
      ETH_NETWORKCFG_MULTICASTHASHEN;

  // Set Full duplex 100Base_TX mode
  ETH->NETWORKCFG |= ETH_NETWORKCFG_FULLDUPLEX | ETH_NETWORKCFG_SPEED;

  //ETH->DMACFG = 0x011A0F10;

  // Set up the ETH DMA with the RX description queue start address
  ETH->RXQPTR = (uint32_t) &dma_descriptor_list.ReceiveBufferQueues[0];

  // Set up the ETH DMA with the TX description queue start address
  ETH->TXQPTR = (uint32_t) &dma_descriptor_list.TransmitBufferQueues[0];

  // Enable transmitter and receiver
  ETH->NETWORKCTRL |= ETH_NETWORKCTRL_ENBRX |
    ETH_NETWORKCTRL_ENBTX |
    ETH_NETWORKCTRL_MANPORTEN;

  // Reset the phy so it's in a known good configuration
  ETH_ResetPhy();

  // Wait for the phy to get a network link
  while(ETH_GetPhyLinkState() == 0)
  {

  }

  // Create the RX stack
  OSTaskCreate(&rx_task_tcb,
               "rx",
               ETH_RXHandler,
               NULL,
               0,
               &rx_task_stk[0],
               (THREADING_CONFIG_RX_TASK_SIZE/10),
               THREADING_CONFIG_RX_TASK_SIZE,
               0,
               0,
               NULL,
               OS_OPT_TASK_STK_CLR,
               &err);

  // Start LWIP
  LWIP_Initalize();

  // Enable interrupts
  ETH_IRQEnable();

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

void ETH_ResetPhy(void){
  // Reset phy command
  ETH->PHYMNGMNT = 0x50028000;
  // Wait for the MDIO operation to complete by watching the flag
  while((ETH->NETWORKSTATUS & ETH_NETWORKSTATUS_MANDONE) == 0)
  {
      __asm("nop");
  }
  RTOS_ERR err;
  // Give the phy some time to reset
  OSTimeDly(100, OS_OPT_TIME_DLY, &err);
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

void LWIP_Initalize(){
  // Start the TCPIP thread
  tcpip_init(NULL, NULL);

  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;

  // Set the IP address structs up for LWIP to consume
  IP4_ADDR(&ipaddr, 10, 10, 5, 200);
  IP4_ADDR(&netmask, 255, 255, 255, 0);
  IP4_ADDR(&gw, 10, 10, 4, 1);

  // Add the internet netif to LWIP. The initialization callback will be called during this function.
  netif_add(&internet_netif, &ipaddr, &netmask, &gw, NULL, &Eth_InitalizeInternetInterface, &tcpip_input);

  // Set the internet interface as the default
  netif_set_default(&internet_netif);

  // Set the interface as up (LWIP can use it)
  netif_set_up(&internet_netif);
}

err_t Eth_InitalizeInternetInterface(struct netif *netif)
{
    // Set the name
    memcpy(netif->name, "eth0", strlen("eth0"));
    // Set the output function
    netif->output = etharp_output;
    // Set the link output function
    netif->linkoutput = ETH_Output;

    // Set the hardware address length
    netif->hwaddr_len = 6;

    // Copy the hardware address from the low level driver to the netif so that LWIP is aware of our mac address.
    memcpy(netif->hwaddr, mac_address, 6);

    // Set netif maximum transfer unit
    netif->mtu = 1500;

    // Accept broadcast address and ARP traffic
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

    // Set netif link flag
    netif->flags |= NETIF_FLAG_LINK_UP;

    return ERR_OK;
}

void ETH_PbufFree(struct pbuf *p)
{
  SYS_ARCH_DECL_PROTECT(old_level);
  // Get the pbuf from the function args
  RxBuff_t* pbuf = (RxBuff_t*)p;
  // Protect from concurrent access
  SYS_ARCH_PROTECT(old_level);
  // We need to set the ownership flag back to 0 so the ETH module can use this buffer again
  dma_descriptor_list.ReceiveBufferQueues[pbuf->dma_descriptor_index].addr &= (~(1 << 0));
  // Release the pbuf back to the pool
  LWIP_MEMPOOL_FREE(RX_POOL, pbuf);
  // Stop protecting from concurrent access
  SYS_ARCH_UNPROTECT(old_level);
}

void ETH_RXHandler(void* param){
  (void) param;
  RTOS_ERR err;
  while(1){
    // Wait for the ETH IRQ to tell us a frame is ready
    OSSemPend(&rx_semaphore, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    // Check all of our RX descriptors to see which ones contain valid frames
    for(int i = 0; i < RX_DESCRIPTOR_QUEUE_SIZE; i++){
        // Check if the buffer has been used by the DMA engine
        if((dma_descriptor_list.ReceiveBufferQueues[i].addr & 0x01) > 0){
            // Uncomment the following for RX descriptor info
            //printf("RX descriptor at index %d has data\r\n", i);

            // Get a PBUF to use for frame reception
            RxBuff_t* p  = (RxBuff_t*)LWIP_MEMPOOL_ALLOC(RX_POOL);
            // Set the funciton that will be called when LWIP is done processing the frame
            p->pbuf_custom.custom_free_function = ETH_PbufFree;
            // Set the DMA descriptor that needs to be processed
            p->dma_descriptor_index = i;
            // Extract the length of the frame
            uint32_t rx_len = dma_descriptor_list.ReceiveBufferQueues[i].status & (uint32_t)0x1FFF;
            uint32_t addr_int = dma_descriptor_list.ReceiveBufferQueues[i].addr & 0xFFFFFFFC;
            void* addr = (void*) addr_int;
            // Create the pbuf reference structure that will be sent to LWIP
            struct pbuf* lwip_pbuf = pbuf_alloced_custom(PBUF_RAW,
                                                         rx_len,
                                                         PBUF_REF,
                                                         &p->pbuf_custom,
                                                         addr,
                                                         NET_IF_ETHER_FRAME_MAX_SIZE);
            // Send the frame to LWIP
            if(netif_input(lwip_pbuf, &internet_netif) != ERR_OK){
                // Failed to process for some reason so we free the pbuf
                pbuf_free(lwip_pbuf);
            }
        }
    }
  }
}


err_t ETH_Output(struct netif *netif, struct pbuf *p)
{
  LWIP_UNUSED_ARG(netif);
  struct pbuf *q;


  // Theoretically, we should never have an ethernet frame expand past 1 pbuf or 1 dma descriptor.
  // That is, LWIP is configured to always have a 1-to-1 relationship with Ethernet frames, pbufs and dma descriptors.

  // Uncomment the following for TX descriptor debugging
  /*
    printf("Printing TX Descriptor status\r\n");
    for(int i = 0; i < TX_DESCRIPTOR_QUEUE_SIZE; i++){
        uint32_t status = dma_descriptor_list.TransmitBufferQueues[i].status;
        printf("\tDMA Descriptor %d status: 0x%08X\r\n", i, status);
    }
    printf("\tTXQPTR: 0x%08X\r\n", ETH->TXQPTR);
    printf("TX Descriptor Index: %lu, \r\n", dma_idx);
 */

  for(q = p; q != NULL; q = q->next) {
      if((dma_descriptor_list.TransmitBufferQueues[dma_idx].status & 0x80000000) > 0){
        // If not, we can use it
        dma_descriptor_list.TransmitBufferQueues[dma_idx].addr = (uint32_t)q->payload;
        // Create a variable that is used to build the status word for this descriptor
        // Note: We are not clearing bits that should be zero because the status word starts with all cleared
        // so make sure you look at table 40.10 to determine what bits should or should not be set
        uint32_t status_word = 0x00;
        // Check if we are at the last descriptor
        if(dma_idx == TX_DESCRIPTOR_QUEUE_SIZE - 1){
            // Last descriptor so we need to set the wrap bit
            status_word |= (1 << 30);
            printf("Last descriptor reached\r\n");
        }

        // If LWIP is configured to generate CRCs in software, then bit 16 needs to be set
        //status_word |= (1 << 16);

        // Check if this is the last pbuf in the chain
        if(q->next == NULL){
            // Set the end of packet
            status_word |= (1 << 15);
        }

        // Add the length of this part of the pbuf chain
        status_word |= ((q->len) & 0x3FFF);

        dma_descriptor_list.TransmitBufferQueues[dma_idx].status = status_word;

        // Move to the next available buffer
        if(dma_idx + 1 == TX_DESCRIPTOR_QUEUE_SIZE)
        {
            // Wrap around if we have to
            dma_idx = 0;
        }else{
            // Move to the next available buffer
            dma_idx++;
        }
      }else{
          // Hmm, something aint right chief
          EFM_ASSERT(0);
          return ERR_MEM;
      }
  }

  ETH->NETWORKCTRL |= ETH_NETWORKCTRL_TXSTRT;

  return ERR_OK;
}

void ETH_IRQHandler(void)
{
  RTOS_ERR err;
  if(ETH->IFCR & ETH_IFCR_RXCMPLT){
     // A packet was received
     OSSemPost(&rx_semaphore, OS_OPT_POST_FIFO, &err);
  }
  // Clear pending ISRs
  ETH->IFCR = 0xFFFFFF;
}

void ETH_IRQEnable(void)
{
  NVIC_EnableIRQ(ETH_IRQn);
}

void ETH_IRQDisable(void)
{
  NVIC_DisableIRQ(ETH_IRQn);
}
