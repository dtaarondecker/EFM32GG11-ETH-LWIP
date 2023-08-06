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
#define TCPIP_THREAD_PRIO               5 //Highest priority - probably not required

/*----- Default value in ETH configuration 1524 -----*/
#define ETH_RX_BUFFER_SIZE 1536



/**
 * Debugging support
 */
#define LWIP_DEBUG LWIP_DBG_ON


/*
   ---------------------------------------
   ---------- Debugging options ----------
   ---------------------------------------
*/
/**
 * @defgroup lwip_opts_debugmsg Debug messages
 * @ingroup lwip_opts_debug
 * @{
 */
/**
 * LWIP_DBG_MIN_LEVEL: After masking, the value of the debug is
 * compared against this value. If it is smaller, then debugging
 * messages are written.
 * @see debugging_levels
 */
#if !defined LWIP_DBG_MIN_LEVEL || defined __DOXYGEN__
#define LWIP_DBG_MIN_LEVEL              LWIP_DBG_LEVEL_ALL
#endif

/**
 * LWIP_DBG_TYPES_ON: A mask that can be used to globally enable/disable
 * debug messages of certain types.
 * @see debugging_levels
 */
#if !defined LWIP_DBG_TYPES_ON || defined __DOXYGEN__
#define LWIP_DBG_TYPES_ON               LWIP_DBG_OFF
#endif

/**
 * ETHARP_DEBUG: Enable debugging in etharp.c.
 */
#if !defined ETHARP_DEBUG || defined __DOXYGEN__
#define ETHARP_DEBUG                    LWIP_DBG_OFF
#endif

/**
 * NETIF_DEBUG: Enable debugging in netif.c.
 */
#if !defined NETIF_DEBUG || defined __DOXYGEN__
#define NETIF_DEBUG                     LWIP_DBG_OFF
#endif

/**
 * PBUF_DEBUG: Enable debugging in pbuf.c.
 */
#if !defined PBUF_DEBUG || defined __DOXYGEN__
#define PBUF_DEBUG                      LWIP_DBG_OFF
#endif

/**
 * API_LIB_DEBUG: Enable debugging in api_lib.c.
 */
#if !defined API_LIB_DEBUG || defined __DOXYGEN__
#define API_LIB_DEBUG                   LWIP_DBG_OFF
#endif

/**
 * API_MSG_DEBUG: Enable debugging in api_msg.c.
 */
#if !defined API_MSG_DEBUG || defined __DOXYGEN__
#define API_MSG_DEBUG                   LWIP_DBG_OFF
#endif

/**
 * SOCKETS_DEBUG: Enable debugging in sockets.c.
 */
#if !defined SOCKETS_DEBUG || defined __DOXYGEN__
#define SOCKETS_DEBUG                   LWIP_DBG_OFF
#endif

/**
 * ICMP_DEBUG: Enable debugging in icmp.c.
 */
#if !defined ICMP_DEBUG || defined __DOXYGEN__
#define ICMP_DEBUG                      LWIP_DBG_OFF
#endif

/**
 * IGMP_DEBUG: Enable debugging in igmp.c.
 */
#if !defined IGMP_DEBUG || defined __DOXYGEN__
#define IGMP_DEBUG                      LWIP_DBG_OFF
#endif

/**
 * INET_DEBUG: Enable debugging in inet.c.
 */
#if !defined INET_DEBUG || defined __DOXYGEN__
#define INET_DEBUG                      LWIP_DBG_OFF
#endif

/**
 * IP_DEBUG: Enable debugging for IP.
 */
#if !defined IP_DEBUG || defined __DOXYGEN__
#define IP_DEBUG                        LWIP_DBG_ON
#endif

/**
 * IP_REASS_DEBUG: Enable debugging in ip_frag.c for both frag & reass.
 */
#if !defined IP_REASS_DEBUG || defined __DOXYGEN__
#define IP_REASS_DEBUG                  LWIP_DBG_OFF
#endif

/**
 * RAW_DEBUG: Enable debugging in raw.c.
 */
#if !defined RAW_DEBUG || defined __DOXYGEN__
#define RAW_DEBUG                       LWIP_DBG_OFF
#endif

/**
 * MEM_DEBUG: Enable debugging in mem.c.
 */
#if !defined MEM_DEBUG || defined __DOXYGEN__
#define MEM_DEBUG                       LWIP_DBG_OFF
#endif

/**
 * MEMP_DEBUG: Enable debugging in memp.c.
 */
#if !defined MEMP_DEBUG || defined __DOXYGEN__
#define MEMP_DEBUG                      LWIP_DBG_OFF
#endif

/**
 * SYS_DEBUG: Enable debugging in sys.c.
 */
#if !defined SYS_DEBUG || defined __DOXYGEN__
#define SYS_DEBUG                       LWIP_DBG_OFF
#endif

/**
 * TIMERS_DEBUG: Enable debugging in timers.c.
 */
#if !defined TIMERS_DEBUG || defined __DOXYGEN__
#define TIMERS_DEBUG                    LWIP_DBG_OFF
#endif

/**
 * TCP_DEBUG: Enable debugging for TCP.
 */
#if !defined TCP_DEBUG || defined __DOXYGEN__
#define TCP_DEBUG                       LWIP_DBG_OFF
#endif

/**
 * TCP_INPUT_DEBUG: Enable debugging in tcp_in.c for incoming debug.
 */
#if !defined TCP_INPUT_DEBUG || defined __DOXYGEN__
#define TCP_INPUT_DEBUG                 LWIP_DBG_OFF
#endif

/**
 * TCP_FR_DEBUG: Enable debugging in tcp_in.c for fast retransmit.
 */
#if !defined TCP_FR_DEBUG || defined __DOXYGEN__
#define TCP_FR_DEBUG                    LWIP_DBG_OFF
#endif

/**
 * TCP_RTO_DEBUG: Enable debugging in TCP for retransmit
 * timeout.
 */
#if !defined TCP_RTO_DEBUG || defined __DOXYGEN__
#define TCP_RTO_DEBUG                   LWIP_DBG_OFF
#endif

/**
 * TCP_CWND_DEBUG: Enable debugging for TCP congestion window.
 */
#if !defined TCP_CWND_DEBUG || defined __DOXYGEN__
#define TCP_CWND_DEBUG                  LWIP_DBG_OFF
#endif

/**
 * TCP_WND_DEBUG: Enable debugging in tcp_in.c for window updating.
 */
#if !defined TCP_WND_DEBUG || defined __DOXYGEN__
#define TCP_WND_DEBUG                   LWIP_DBG_OFF
#endif

/**
 * TCP_OUTPUT_DEBUG: Enable debugging in tcp_out.c output functions.
 */
#if !defined TCP_OUTPUT_DEBUG || defined __DOXYGEN__
#define TCP_OUTPUT_DEBUG                LWIP_DBG_OFF
#endif

/**
 * TCP_RST_DEBUG: Enable debugging for TCP with the RST message.
 */
#if !defined TCP_RST_DEBUG || defined __DOXYGEN__
#define TCP_RST_DEBUG                   LWIP_DBG_OFF
#endif

/**
 * TCP_QLEN_DEBUG: Enable debugging for TCP queue lengths.
 */
#if !defined TCP_QLEN_DEBUG || defined __DOXYGEN__
#define TCP_QLEN_DEBUG                  LWIP_DBG_OFF
#endif

/**
 * UDP_DEBUG: Enable debugging in UDP.
 */
#if !defined UDP_DEBUG || defined __DOXYGEN__
#define UDP_DEBUG                       LWIP_DBG_OFF
#endif

/**
 * TCPIP_DEBUG: Enable debugging in tcpip.c.
 */
#if !defined TCPIP_DEBUG || defined __DOXYGEN__
#define TCPIP_DEBUG                     LWIP_DBG_OFF
#endif

/**
 * SLIP_DEBUG: Enable debugging in slipif.c.
 */
#if !defined SLIP_DEBUG || defined __DOXYGEN__
#define SLIP_DEBUG                      LWIP_DBG_OFF
#endif

/**
 * DHCP_DEBUG: Enable debugging in dhcp.c.
 */
#if !defined DHCP_DEBUG || defined __DOXYGEN__
#define DHCP_DEBUG                      LWIP_DBG_OFF
#endif

/**
 * AUTOIP_DEBUG: Enable debugging in autoip.c.
 */
#if !defined AUTOIP_DEBUG || defined __DOXYGEN__
#define AUTOIP_DEBUG                    LWIP_DBG_OFF
#endif

/**
 * DNS_DEBUG: Enable debugging for DNS.
 */
#if !defined DNS_DEBUG || defined __DOXYGEN__
#define DNS_DEBUG                       LWIP_DBG_OFF
#endif

/**
 * IP6_DEBUG: Enable debugging for IPv6.
 */
#if !defined IP6_DEBUG || defined __DOXYGEN__
#define IP6_DEBUG                       LWIP_DBG_OFF
#endif

/**
 * DHCP6_DEBUG: Enable debugging in dhcp6.c.
 */
#if !defined DHCP6_DEBUG || defined __DOXYGEN__
#define DHCP6_DEBUG                     LWIP_DBG_OFF
#endif


#endif /* CONFIG_LWIPOPTS_H_ */
