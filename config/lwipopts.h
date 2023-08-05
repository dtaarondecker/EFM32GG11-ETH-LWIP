/*
 * lwipopts.h
 *
 *  Created on: Aug 3, 2023
 *      Author: adeck
 */

#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#define NO_SYS                  0

/* Memory options */
#define MEM_ALIGNMENT           4

/* the size of the heap memory.  */
#define MEM_SIZE                (20 * 1024)

/* the number of memp struct pbufs. */
#define MEMP_NUM_PBUF           10
/* the number of UDP protocol control blocks. One per active UDP "connection". */
#define MEMP_NUM_UDP_PCB        6
/* the number of simultaneously active TCP connections. */
#define MEMP_NUM_TCP_PCB        10
/* the number of listening TCP connections. */
#define MEMP_NUM_TCP_PCB_LISTEN 5
/* the number of simultaneously queued TCP segments. */
#define MEMP_NUM_TCP_SEG        16
/*  the number of simultaneously active timeouts. */
#define MEMP_NUM_SYS_TIMEOUT    10

// pbuf options
/* the number of buffers in the pbuf pool. */
#define PBUF_POOL_SIZE          10

/* the size of each pbuf in the pbuf pool. */
#define PBUF_POOL_BUFSIZE       1582

/* TCP options  */
#define LWIP_TCP                1
#define TCP_TTL                 255

/* Controls if TCP should queue segments that arrive out of
   order. Define to 0 if your device is low on memory. */
#define TCP_QUEUE_OOSEQ         1

/* TCP Maximum segment size. */
#define TCP_MSS                 (1500 - 40)

/* TCP sender buffer space (bytes). */
#define TCP_SND_BUF             (8 * TCP_MSS)

/*  TCP sender buffer space (pbufs). This must be at least
   as much as (2 * TCP_SND_BUF/TCP_MSS) for things to work. */
#define TCP_SND_QUEUELEN        (2 * TCP_SND_BUF / TCP_MSS)

/* TCP receive window. */
#define TCP_WND                 (8 * TCP_MSS)

/* ICMP options */
#define LWIP_ICMP                       1

/* DHCP options */
#define LWIP_DHCP               1
#define ETHARP_SUPPORT_STATIC_ENTRIES 1

// Enable/disable Netconn API (require to use api_lib.c)
#define LWIP_NETCONN                    1

// Enable/disable Socket API (require to use sockets.c)
#define LWIP_SOCKET                     1

// OS related options
#define TCPIP_THREAD_NAME              "TCP/IP"
#define TCPIP_THREAD_STACKSIZE          2048
#define TCPIP_MBOX_SIZE                 10
#define DEFAULT_UDP_RECVMBOX_SIZE       10
#define DEFAULT_TCP_RECVMBOX_SIZE       10
#define DEFAULT_ACCEPTMBOX_SIZE         10
#define DEFAULT_THREAD_STACKSIZE        2048
#define TCPIP_THREAD_PRIO               0 //Highest priority - probably not required

/*----- Default value in ETH configuration 1524 -----*/
#define ETH_RX_BUFFER_SIZE 1536

// Checksum options - All software
#define CHECKSUM_GEN_IP                 1
#define CHECKSUM_GEN_UDP                1
#define CHECKSUM_GEN_TCP                1
#define CHECKSUM_GEN_ICMP               1
#define CHECKSUM_CHECK_IP               1
#define CHECKSUM_CHECK_UDP              1
#define CHECKSUM_CHECK_TCP              1
#define CHECKSUM_CHECK_ICMP             1


#endif /* CONFIG_LWIPOPTS_H_ */
