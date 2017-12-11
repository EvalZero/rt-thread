#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

/* Automatically generated file; DO NOT EDIT. */
/* RT-Thread Configuration */

/* RT-Thread Kernel */

#define RT_NAME_MAX 8
#define RT_ALIGN_SIZE 8
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 100
#define RT_DEBUG
#define RT_USING_OVERFLOW_CHECK
#define RT_DEBUG_INIT 1
#define RT_DEBUG_THREAD 0
//#define RT_USING_HOOK
#define IDLE_THREAD_STACK_SIZE 512
#define RT_USING_TIMER_SOFT
#define RT_TIMER_THREAD_PRIO 4
#define RT_TIMER_THREAD_STACK_SIZE 512

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE

/* Memory Management */

#define RT_USING_MEMPOOL
#define RT_USING_MEMHEAP
#define RT_USING_HEAP
/* RT_USING_SMALL_MEM is not set */
#define RT_USING_SLAB

/* Kernel Device Object */

#define RT_USING_DEVICE
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 512
#define RT_CONSOLE_DEVICE_NAME "uart0"
/* RT_USING_MODULE is not set */

/* RT-Thread Components */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN

/* C++ features */

//#define RT_USING_CPLUSPLUS

/* Command shell */

#define RT_USING_FINSH
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
#define FINSH_THREAD_STACK_SIZE 4096
/* FINSH_USING_AUTH is not set */
#if 0
#define FINSH_DEFAULT_PASSWORD "rtthread"
#define FINSH_USING_MSH
#define FINSH_USING_MSH_DEFAULT
#define FINSH_USING_MSH_ONLY
#endif
/* Device virtual file system */
#define RT_USING_DFS
#define DFS_USING_WORKDIR
#define DFS_FILESYSTEMS_MAX 4
#define DFS_FD_MAX 4
#define RT_USING_DFS_ELMFAT
#define RT_DFS_ELM_CODE_PAGE 437
#define RT_DFS_ELM_WORD_ACCESS
#define RT_DFS_ELM_USE_LFN_0
/* RT_DFS_ELM_USE_LFN_1 is not set */
/* RT_DFS_ELM_USE_LFN_2 is not set */
/* RT_DFS_ELM_USE_LFN_3 is not set */
#define RT_DFS_ELM_USE_LFN 0
#define RT_DFS_ELM_MAX_LFN 256
#define RT_DFS_ELM_DRIVES 4
#define RT_DFS_ELM_MAX_SECTOR_SIZE 4096
/* RT_DFS_ELM_USE_ERASE is not set */
#define RT_DFS_ELM_REENTRANT
#define RT_USING_DFS_DEVFS

/* RT_USING_DFS_NET is not set */
/* RT_USING_DFS_NFS is not set */

/* Network stack */

/* light weight TCP/IP stack */

//#define RT_USING_LWIP
/* RT_USING_LWIP141 is not set */
/* RT_USING_LWIP200 is not set */
#define RT_USING_LWIP202
#define RT_LWIP_IGMP
#define RT_LWIP_ICMP
/* RT_LWIP_SNMP is not set */
#define RT_LWIP_DNS
//#define RT_LWIP_DHCP
#define IP_SOF_BROADCAST 1
#define IP_SOF_BROADCAST_RECV 1
#define LWIP_USING_DHCPD
#define RT_LWIP_UDP
#define RT_LWIP_TCP
/* RT_LWIP_RAW is not set */
/* RT_LWIP_PPP is not set */
/* RT_LWIP_PPPOE is not set */
/* RT_LWIP_PPPOS is not set */
#define RT_LWIP_PBUF_NUM 64
#define RT_LWIP_RAW_PCB_NUM 32
#define RT_LWIP_UDP_PCB_NUM 16
#define RT_LWIP_TCP_PCB_NUM 16
#define RT_LWIP_TCP_SEG_NUM 32
#define RT_LWIP_TCP_SND_BUF (8 * 1024)
#define RT_LWIP_TCP_WND     32768
#define RT_LWIP_TCPTHREAD_PRIORITY 10
#define RT_LWIP_TCPTHREAD_MBOX_SIZE 8
#define RT_LWIP_TCPTHREAD_STACKSIZE 4096
#define RT_LWIP_ETHTHREAD_PRIORITY 12
#define RT_LWIP_ETHTHREAD_STACKSIZE 1024
#define RT_LWIP_ETHTHREAD_MBOX_SIZE 8
/* RT_LWIP_REASSEMBLY_FRAG is not set */
#define LWIP_NETIF_STATUS_CALLBACK 1
#define SO_REUSE 1
#define LWIP_SO_RCVTIMEO 1
#define LWIP_SO_SNDTIMEO 1
#define LWIP_SO_RCVBUF 1

#ifndef RT_LWIP_DHCP
#  define RT_LWIP_IPADDR      "192.168.1.101"
#  define RT_LWIP_GWADDR      "192.168.1.1"
#  define RT_LWIP_MSKADDR     "255.255.255.0"
#endif

/* Modbus master and slave stack */

//#define RT_USING_MODBUS
/* RT_MODBUS_MASTER_RTU is not set */
#define RT_MODBUS_SLAVE_RTU
//#define RT_USING_NETUTILS

/* Device Drivers */

#define RT_USING_DEVICE_IPC
#define RT_USING_SERIAL
#define RT_SERIAL_RB_BUFSZ  2048
/* RT_USING_CAN is not set */
/* RT_USING_HWTIMER is not set */
#define RT_USING_I2C
#define RT_USING_I2C_BITOPS
#define RT_USING_PIN
#define RT_USING_MTD_NOR
#define RT_USING_MTD_NAND
#define RT_MTD_NAND_DEBUG
#define RT_USING_RTC
//#define RT_USING_SDIO
#define RT_USING_SPI
//#define RT_USING_SFUD
#define RT_SFUD_USING_SFDP
#define RT_SFUD_USING_FLASH_INFO_TABLE
#define RT_SFUD_DEBUG
#define RT_USING_W25QXX
/* RT_USING_GD is not set */
/* RT_USING_ENC28J60 is not set */
/* RT_USING_SPI_WIFI is not set */
#define RT_USING_WDT
/* RT_USING_USB_HOST is not set */
//#define RT_USING_USB_DEVICE
#define RT_USB_DEVICE_CDC
#define RT_USB_DEVICE_MSTORAGE

/* RT-Thread UI Engine */

/* RT_USING_GUIENGINE is not set */
#define RT_USING_GUIENGINE
#define RTGUI_NAME_MAX 16
/* RTGUI_USING_TTF is not set */
#define RTGUI_USING_FONT16
#define RTGUI_USING_FONT12
/* RTGUI_USING_FONTHZ is not set */
/* RTGUI_IMAGE_XPM is not set */
#define RTGUI_IMAGE_JPEG_NONE
/* RTGUI_IMAGE_JPEG is not set */
/* RTGUI_IMAGE_TJPGD is not set */
#define RTGUI_IMAGE_PNG_NONE
/* RTGUI_IMAGE_PNG is not set */
/* RTGUI_IMAGE_LODEPNG is not set */
#define RTGUI_IMAGE_LODEPNG
/* RTGUI_IMAGE_BMP is not set */
/* RTGUI_IMAGE_CONTAINER is not set */

/* libc */

#define RT_USING_LIBC
/* RT_USING_PTHREADS is not set */

/* RT-Thread online packages */

/* system packages */

/* PKG_USING_PARTITION is not set */
/* PKG_USING_SQLITE is not set */

/* IoT - internet of things */

/* PKG_USING_PAHOMQTT is not set */
/* PKG_USING_WEBCLIENT is not set */
/* PKG_USING_MONGOOSE is not set */
/* PKG_USING_WEBTERMINAL is not set */
/* PKG_USING_CJSON is not set */
/* PKG_USING_EZXML is not set */

/* Marvell WiFi */

/* PKG_USING_MARVELLWIFI is not set */

/* security packages */

/* PKG_USING_MBEDTLS is not set */

/* language packages */

/* PKG_USING_JERRYSCRIPT is not set */

/* multimedia packages */

/* PKG_USING_FASTLZ is not set */

/* tools packages */

/* PKG_USING_CMBACKTRACE is not set */
/* PKG_USING_EASYLOGGER is not set */

/* miscellaneous packages */

/* PKG_USING_HELLO is not set */

#define RT_USING_SPISPI
#define RT_USING_SPI1
#define RT_USING_W25QXX
#define FLASH_DEBUG


#define RT_USING_RYM

#endif
