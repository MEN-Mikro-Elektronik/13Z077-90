/*********************  P r o g r a m  -  M o d u l e ***********************/
/*!
 *        \file  men_16z077_eth.c
 *
 *      \author  thomas.schnuerer@men.de
 *        $Date: 2014/07/16 19:30:45 $
 *    $Revision: 1.45 $
 *
 *        \brief driver for IP core 16Z087 (Ethernet cores).
 *               supports kernel 2.6 and 3.x
 *
 *     Switches: NIOS_II  			defined for build on Nios II based systems
 *				 CONFIG_MENEM1		defined for build with MEN EM01A/N
 *				 CONFIG_MENEM9      defined for build with MEN EM09
 *  			 CONFIG_MENEP05     defined for build with MEN EP05/6/7
 *				 NO_PHY             defined for specific PHY-less HW
 *				 MEN_MM1            defined for build with MEN MM1
 *
 *
 *
 * DESCRIPTION
 *
 * The driver supports the IP core 16Z087 derived from Opencores ETH MAC
 * ( www.opencores.org )
 *
 * \verbatim
 * Description of Registers:
 *
 * Name          Address     Width   Access  Description
 * -------------------------------------------------------------------------
 * MODER         0x00        32      RW      Mode Register
 * INT_SOURCE    0x04        32      RW      Interrupt Source Register
 * INT_MASK      0x08        32      RW      Interrupt Mask Register
 * IPGT          0x0C        32      RW      Back to Back Inter Packet
 *                                           GapRegister
 * IPGR1         0x10        32      RW      Non Back to Back Inter Packet
 *                                           Gap Register 1
 * IPGR2         0x14        32      RW      Non Back to Back Inter Packet
 *                                           Gap Register 2
 * PACKETLEN     0x18        32      RW      Packet Length (minimum and
 *                                           maximum) Register
 * COLLCONF      0x1C        32      RW      Collision and Retry Configuration
 * TX_BD_NUM     0x20        32      RW      Transmit Buffer Descriptor Number
 * CTRLMODER     0x24        32      RW      Control Module Mode Register
 * MIIMODER      0x28        32      RW      MII Mode Register
 * MIICOMMAND    0x2C        32      RW      MII Commend Register
 * MIIADDRESS    0x30        32      RW      MII Address Register Contains
 *                                           the PHY address and the register
 *  										 within the PHY address
 * MIITX_DATA    0x34        32      RW      MII Transmit Data The data to
 *                                           be transmitted to the PHY
 * MIIRX_DATA    0x38        32      RW      MII Receive Data The data
 *                                           received from the PHY
 * MIISTATUS     0x3C        32      RW      MII Status Register
 *
 * MAC_ADDR0     0x40        32      RW      MAC Individual Address0 The LSB
 *                                           four bytes of the MAC address
 * MAC_ADDR1     0x44        32      RW      MAC Individual Address1 The MSB
 *                                           two bytes of the MAC
 * ETH_HASH0_ADR 0x48        32      RW      HASH0 Register
 * ETH_HASH1_ADR 0x4C        32      RW      HASH1 Register
 * ETH_TXCTRL    0x50        32      RW      Transmit Control Register
 *
 *
 * original ETH OpenCore:  BDs start at Z077_BD_OFFS (0x400) after ChamBase.
 *
 * Z077_BDBASE  = ChamBase + 0x400
 * Z077_BD_OFFS = 0x400
 * +--------------+
 * |			  |
 * |--------------|- Rx BD Start
 * |   	   	   	  |
 * |--------------|- TX Bd  Start (= Z077_BDBASE + Z077_BD_OFFS )
 * |XXXXXXXXXXXXXX|  Ctrl Regs
 * +--------------+- Chameleon Base Address
 *
 *
 * New Core: BDs reside in RAM, Address must be 10bit-aligned
 *
 * Z077_BDBASE  = pDrvCtrl->bdBase;
 * Z077_BD_OFFS = 0
 *
 * +--------------+
 * |			  |
 * |--------------|- Tx BD Start
 * |   	   	   	  |
 * +--------------+- Rx Bd  Start (= Z077_BDBASE + Z077_BD_OFFS )
 *
 *
 *\endverbatim
 * For tests with RAM based images do
 * route add -net 192.168.1.0 netmask 255.255.255.0 dev eth0 gw 192.1.1.22
 *
 */
/*-------------------------------[ History ]---------------------------------
 * 
 * ts 18.05.2015  fixed ETHTOOL_OPS removement from 3.16 on
 *
 * ------------- end of cvs controlled source -------------------
 * $Log: men_16z077_eth.c,v $
 * Revision 1.45  2014/07/16 19:30:45  ts
 * R: 1. Compilerwarning: incompatible pointer type of .ndo_vlan_rx_add_vid
 *    2. several compilerwarnings about typecasts with gcc 4.8, kernel 3.14
 * M: 1. corrected declaration of z77_vlan_rx_add_vid
 *    2. changed variable types to appropriate ones, use unsigned long etc.
 *
 * Revision 1.44  2014/06/11 12:14:37  ts
 * R: from kernel 3.10 on build failed because of undefined symbols
 * M: replace NETIF_xxx flags for VLAN support with xxx_CTAG_ as in 3.10.0 and up
 *
 * Revision 1.43  2013/11/06 11:18:37  ts
 * R: under Arch linux 3.10.10 the board ID EEPROM wasnt found on F011C
 * M: increased I2C_MAX_ADAP_CNT to 32
 *
 * Revision 1.42  2013/10/02 13:30:57  ts
 * R: on F75P transfers were hanging under heavy load
 * M: corrected typo in z77_irq (all IRQs except RXE were ack'ed, not RXF)
 *
 * Revision 1.41  2013/08/20 15:26:57  ts
 * R: during tests on F75P multiple instances of netio etc. stopped working
 * M: in send_packet() free skb buffers also before return due to TX busy
 *
 * Revision 1.40  2013/07/10 18:16:48  ts
 * R: minimum frame len 0x40 made problem in customer application due
 *    to PACKLEN register interpreting the value without FCS
 * M: changed header file, reverted this files last work checking for 
 *    other CPU back to 1.38 to avoid other side effects
 *
 * Revision 1.39  2013/04/19 17:41:40  ts
 * R: work checkin for F75P test
 *
 * Revision 1.38  2013/03/23 14:13:23  ts
 * R: checkin bug during last revision caused spin unlocking 2 times
 * M: removed doubled line in ethtool info get function
 *
 * Revision 1.37  2013/03/08 17:40:54  ts
 * R: 1. upon TX timeouts driver could hang
 *    2. very high IRQ load (broadcast storms) could cause crash
 * M: 1. move ndo_tx_timeout handler into process context
 *    2. reworked IRQ masking/reenabling in z77_irq
 *
 * Revision 1.36  2012/11/06 20:17:14  ts
 * R: 1. support for Multicast reception requested by customer
 *    2. MAC addrs for x86 boards were not generated according to ident EEprom
 *    3. build failed for kernel versions > 3.0
 * M: 1. added hash CRC calculation and hash bin setting
 *    2. added detection of board ident EEPROMs (currently F11S / P51x) and
 *       MAC generation. Needs men_lx_z001 loaded for P51x.
 *    3. added API changes (multicast, VLAN processing) for kernel up to 3.6
 *
 * Revision 1.35  2012/10/01 11:57:38  ts
 * R: 1. MAC Address of Z87 instances on different FPGAs/boards were not handled
 *    2. cosmetics, debug messages had no identical look
 * M: 1. added detection of CPU board type and FPGA table readout to assign MAC
 *       as specified in ETH mac numbers document
 *    2. added driver identifier as prefix to all debug printouts
 *
 * Revision 1.34  2012/09/24 14:23:15  ts
 * R: packet length reported on raw sockets was 4 byte too large
 * M: subtract length of CRC bytes from pkt_len
 *
 * Revision 1.33  2012/09/20 18:32:25  ts
 * R: 1. Multicast support was requested by customer
 *    2. mode parameter at modprobe is used for every instance, causing link
 *       problems when link needs to be renegotiated on one interface
 * M: 1. added CRC hashing, processing of mcast MACs passed to driver
 *    2. added parsing of comma separated Phy modes
 *
 * Revision 1.32  2012/09/07 18:03:33  ts
 * R: support for IEEE 802.1Q VLAN tagging requested by customer
 * M: 1. added capability to process VLAN tagged frames
 *    2. updated driver documentation
 *
 * Revision 1.31  2012/03/28 18:21:14  ts
 * R: support for phy driver platform necessary on EP05
 * M: 1. added members mii_bus and phy_device to z77_private struct
 *    2. added z77_mii_init/probe functions to fill mii_bus members and init phy
 *    3. added declaration compatible wrappers for z77_mdio_read/write
 *       to pass to mii_bus as read/write function
 *    4. define MEN_Z77_USE_OWN_PHYACCESS defined in .h file depending on kernel
 *       version (phy driver platform introduced from 2.6.28 on). Determines if
 *       previous builtin phy access or phy driver functions are used
 *
 * Revision 1.30  2011/06/08 18:01:32  rt
 * R: 1) Changing MAC addr. did not work.
 *    2) Settings can be changed even if interface is down (changes
 *       will be lost at interface up).
 *    3) Statistics improved.
 *    4) Under some special circumstates changing settings may not work
 *       correctly.
 *    5) Reduce output of driver messages.
 *    6) Interface restart after TX time-out may fail.
 *    7) Under high load a lot of packets are dropped.
 *    8) Z77: The empty buffer verification in z77_send_packet() is wrong.
 * M: 1.a) Z87 must be disabled to change MAC addr. (see z77_set_mac_address()).
 *      b) Don't overwrite MAC addr. at interface up.
 *    2) IFF_UP flag implemented.
 *    3.a) Added support for detailed RX/TX error counters.
 *      b) Increase the error counter only one time per error.
 *    4) Force link-up after setting settings (see z77_ethtool_set_settings()).
 *    5) Changed some kernel messages to driver debug messages.
 *    6) Restore settings after restart.
 *    7) Check if there is a free TBD for the next TX packet. If not stop
 *       TX queue timely.
 *    8) Negation added.
 *
 * Revision 1.29  2011/05/03 17:46:47  cr
 * R: 1. could not set MAC adress with ifconfig
 * M: 1. implemented z77_set_mac_address()
 *
 * Revision 1.28  2011/04/21 14:23:00  cr
 * R: 1. support ethtool for PHYless communication protocols (e.g. EP05 GPSI)
 * M: 1a) in z77_ethtool_get_settings(), read duplexity settings from Z087 registers and hand
 *        over to ethtool if we are in NO_PHY mode
 *    1b) in z77_ethtool_set_settings(), set duplexity in Z087 registers without reading it back
 *        from the PHY in case we have NO_PHY
 *
 * Revision 1.27  2011/03/16 17:59:10  ts
 * R: driver did not support promiscuous mode of Z087 core
 * M: added processing IFF_PROMISC flag passed from network applications
 *
 * Revision 1.26  2011/02/23 15:10:47  rt
 * R: 1) FPGA units set-up to old IRQ behavior may freeze the Linux system.
 *    2) If all RXBDs are full the Linux system may freeze.
 * M: 1.a) Force new IRQ behavior.
 *      b) Changed i to signed int in z77_process_rx().
 *      c) Print warning if z77_process_rx() is entered while all RXBDs are empty.
 *    2.a) Changed i to signed int in z77_process_rx().
 *      b) Try to use RXBDSTAT register in this case.
 *
 * Revision 1.25  2010/12/20 11:59:23  cr
 * R: changes of revision 1.24 were based on old revision 1.21
 * M: merged changes of revision 1.24 with revision 1.23
 *
 * Revision 1.23  2010/11/25 17:27:40  ts
 * R: link detection not reliably possible with ethtool
 * M: added sysfs file /sys/class/net/ethX/linkstate to read BMSR
 *
 * Revision 1.22  2010/11/18 09:23:23  cr
 * R: 1. support for EP05
 *    2. cosmetics
 * M: 1a) include fdt read/write support (of_platform.h) for EP05
 *    1b) define NO_PHY in case GPSI mode is configured
 *    1c) takeover MAC and PHY address from fdt
 *    2a) only declare writemac if MEN_MM1 is defined (caused compiler warning)
 *    2b) in z77_set_mac_address: commented out variable nic (caused compiler warning)
 *
 * Revision 1.21  2010/10/14 13:21:26  ts
 * R: module rmmod and re-modprobe caused oops
 * M: NAPI change introduced in 1.12 did not disable NAPI in module exit, fixed
 *
 * Revision 1.20  2010/08/20 19:46:01  rt
 * R: 1) Support for EM9A.
 *    2) Cosmetics.
 * M: 1) a) Added MAC determination.
 *       b) Set default PHY addresses.
 *    2) Warning if unclear how to build MAC address, etc.
 *
 * Revision 1.19  2010/06/04 11:45:15  ts
 * R: 1. new PHYs: Broadcom 54xx on F11S, MEN dummy PHY in F218 FPGA
 *    2. driver failed to compile with kernels > 2.6.30 because API changed
 * M: 1. added PHY IDs in PHY table
 *    2. added support for struct net_device_ops
 *
 * Revision 1.18  2010/02/19 16:37:52  ts
 * R: deprecated SHIRQ flag causes warning in 2.6.22 already
 * M: use new IRQF_SHARED in 2.6.19 and up (was introduced in 2.6.18)
 *
 * Revision 1.17  2010/01/25 16:33:41  rt
 * R: 1) Cosmetics
 * M: 1) Avoid compiler warning at printk.
 *
 * Revision 1.16  2009/09/17 15:07:18  ts
 * R: 1) Link changes were not processed correctly and displayed
 *    2) under heavy load unprocessed RX BDs might occur
 * M: 1) Added periodically link status check and adjustment of PHY settings
 *    2) reworked algorithm to process RX BDs so none can be left
 *
 * Revision 1.15  2009/06/09 14:40:23  ts
 * R: 1. 16Z087 not working correct currently (PCIe and FIFO errors)
 *    2. when TX timeout occured (NETDEV_WDOG timeout) NIC was
 *       not restarted properly
 * M: 1. intermediate working checkin done
 *    2. corrected z77_tx_timeout to call z77_close()
 *
 * Revision 1.14  2009/02/18 14:19:46  GLeonhardt
 * R:1. file not compileable for NIOS2
 *   2. no memory for bdBase alignement
 * M:1. remove SYS_Param_Get
 *   2. alloc memory for alignment
 *
 * Revision 1.13  2009/02/10 19:17:18  ts
 * R: additional support for 15P511 was necessary
 * M: added parameters nodma and multiple instance support
 *
 * Revision 1.12  2008/07/14 11:08:24  aw
 * R: kernel napi interface changed at version 2.6.23
 * M: adapted ethernet driver
 *
 * Revision 1.11  2008/06/27 14:44:04  aw
 * R: Sysparam library was needed at F302
 * M: Added SysParamGet to get serial number, MAC address and PHY address
 *
 * Revision 1.10  2008/04/23 17:51:50  ts
 * - made NIOS_II the only depending switch
 * - renamed message"Z77 initialised" into generic "ETH core initialized"
 *
 * Revision 1.9  2008/04/03 17:02:05  aw
 * bugfix - global reset not available at 16Z087, disable RX and TX to reset
 *
 * Revision 1.8  2008/03/20 14:50:12  aw
 * bugfix - declared same variable twice at define CONFIG_PCI
 *
 * Revision 1.7  2008/03/13 18:26:02  aw
 * bugfix - don't use PCI - functions if not defined CONFIG_PCI
 *
 * Revision 1.6  2008/03/12 10:18:16  aw
 * supports NIOS_II in combination with Z087
 *
 * Revision 1.5  2008/02/14 13:31:15  ts
 * bugfix: Broadcast reception in bit OETH_MODER_BRO is enabled if bit=1, not
 *         disabled! added bit to initialization of MODER Register
 *
 * Revision 1.4  2007/11/16 15:53:50  ts
 * Cosmetics, completed doxygen headers
 *
 * Revision 1.3  2007/10/26 16:54:38  ts
 * fixed: set HD/FD bit in MODER[10] according to passed ethtool call
 * cosmetics, doxygen headers cleaned
 *
 * Revision 1.2  2007/10/22 10:20:22  ts
 * made debug dumps available at runtime with ethtool -s eth0 msglvl [0..3]
 * bugfix: call unregister_netdev when module removed, caused oops with ifconfig
 * cosmetics: unified function names
 *            unnecessary defines removed
 *            doxygen function headers added
 *
 * Revision 1.1  2007/10/16 17:29:06  ts
 * Initial Revision
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 2012 by MEN mikro elektronik GmbH, Nuremberg, Germany
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ****************************************************************************/
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/version.h>
#include <linux/ethtool.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/pci.h>
#include <linux/mii.h>

#if defined(CONFIG_MENEP05)
# include <linux/of_platform.h>
#endif

#include <MEN/men_typs.h>
#include <MEN/oss.h>
#include <MEN/mdis_err.h>
#include <MEN/dbg.h>
#include <MEN/men_chameleon.h>
#if defined(MEN_MM1)  /* init SMB2 lib for EEPROM readout */
# include <MEN/smb2.h>
#endif
#include <MEN/sysparam2.h>
#include <MEN/sysdep.h>
#include "men_16z077_eth.h"

/*-----------------------------+
 |  DEFINES                    |
 +-----------------------------*/

/* # of supported Z87 Instances in the system */
#define NR_ETH_CORES_MAX		8
/* timeout to wait for reply from MII */
#define MII_ACCESS_TIMEOUT 		10000
/*  max. loops to wait for Idle condition */
#define IP_CORE_TIMEOUT 		200

/* size and BAR of P511's SDRAM: 32M */
#define P511_BAR1_SDRAMSIZE 	0x2000000
#define P511_SDRAM_BAR			1
#define PHY_MAX_ADR				31

#define MAX_MCAST_LST			64			/* we store up to 64 addresses */
#define MAC_ADDR_LEN			6			/* MAC len in byte */
#define MCAST_HASH_POLYNOM  	0x04C11DB7	/* CRC32 polynom for hash calculation */
#define MCAST_HASH_CRC_SEED 	0xFFFFFFFF	/* initial polynom seed */
#define MCAST_MULT_SHFT 		26			/* bit shift to calculate hash bin */
#define MCAST_HASH_MASK			0x3f    	/* pass up only lower 6 bit */
#define LEN_CRC					4			/* additional CRC bytes present in Frames */
#define I2C_MAX_ADAP_CNT		32			/* max. # of I2C adapters to query ID EEPROM */
#define MEN_F1x_EEID_SMB_ADR   	(0xae>>1)	/* ID EEPROM addres on F1x cards = 0x57 */
#define ID_EE_SERN_OFF 			3			/* serial nr. offset in ID EEPROM */
#define ID_EE_SERN_LEN			4			/* length of serial nr. in ID EEPROM */
#define ID_EE_NAME_OFF 			9			/* board name offset in ID EEPROM */
#define ID_EE_NAME_LEN			6			/* length of name in ID EEPROM */
#define IC_BOARD_NAME_LEN		4			/* length of board in IC file name */

/* all used IRQs on the Z87 */
#define Z077_IRQ_ALL (OETH_INT_TXE | OETH_INT_RXF | OETH_INT_RXE | OETH_INT_BUSY | OETH_INT_TXB)

/* from 3.1 on (acc. to free electrons) DMA bit mask changed */
#if LINUX_VERSION_CODE > VERSION_CODE(3,1,0)
# define Z87_BIT_MASK_32BIT		DMA_BIT_MASK(32)
#else
# define Z87_BIT_MASK_32BIT		DMA_32BIT_MASK
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
# define Z87_VLAN_FEATURES    (NETIF_F_HW_VLAN_CTAG_TX | NETIF_F_HW_VLAN_CTAG_RX)
#else
# define Z87_VLAN_FEATURES    (NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX)
#endif


#ifndef TRUE
# define TRUE 1
#endif
#ifndef FALSE
# define FALSE 0
#endif

#ifdef NIOS_II
# include <asm/cacheflush.h>
# include <asm/uaccess.h>
# define NO_CACHED_MEM 0x80000000
#else
# define NO_CACHED_MEM 0x0
#endif

#define PCIDDR_OFFS 0x80000

#define VLAN_TAG_SIZE			4		/* how much longer VLAN frames are */
#define ETH_SRCDST_MAC_SIZE		12		/* combined length or source and dest MAC addr */


/* For the DMA write physical addresses into BDs are needed. unfortunately
 * they can be in different defines */
#ifdef CONFIG_KERNEL_START
# define VIRTUAL_MEM_OFFSET  CONFIG_KERNEL_START
#else
# ifdef CONFIG_PAGE_OFFSET
# define VIRTUAL_MEM_OFFSET  CONFIG_PAGE_OFFSET
# endif
#endif

/* if none of the above is defined exit to the kernel default Value */
#ifndef VIRTUAL_MEM_OFFSET
#define VIRTUAL_MEM_OFFSET    0xc0000000
#endif

#if defined(CONFIG_MENEP05)
#define MENEP05_Z77_INSTNAME 	"instance"
#define MENEP05_Z77_PHYADR   	"phy-address"
#define MENEP05_Z77_MACADR  	"local-mac-address"
 #if defined(CONFIG_MEN_16Z077_GPSI) && !defined(NO_PHY)
  #define NO_PHY
 #endif
#endif


#ifndef WARN_ONCE
/* if WARN_ONCE not supported, ignore output */
#define WARN_ONCE(condition, format...)
#endif

#define Z77_DEFAULT_MAC 	"ff:ff:ff:ff:ff:ff"

/*
 * Macro for use in conjunction with ethtool -s eth.. msglvl <n>
 * the value <n> can vary from 0(off) to 3 (very verbose plus IRQ dumps)
 */

#ifdef DBG
#define Z77DBG(lvl,msg...) 	  if (np->msg_enable >= lvl) \
                                     printk( msg );
# else
#define Z77DBG(lvl,msg...) 	  do {} while (0)
#endif

/* F218 uses other Micrel PHY */
#define PHY_ID2_KSZ8041_1	0x1512
#define PHY_ID2_KSZ8041_2	0x1513

static OSS_HANDLE		*G_osh;		/* OSS handle */

/* sanity checks, this one isnt really necessary anymore since Z87 */
#if (Z077_TBD_NUM < 1) || (Z077_RBD_NUM < 1)
# error Z077_TBD/RBD_NUM must be at least 1 !
#endif

/* #define DO_NETDEV_TIMEOUT_TEST */
#undef DO_NETDEV_TIMEOUT_TEST

#ifdef DO_NETDEV_TIMEOUT_TEST
#warning "!!!!!!!!!!!!! ATTENTION!!!!!!!!!!!!! Netdev Watchdog timeout test enabled! no production driver!"
#endif

/*--------------------------------+
 |  TYPEDEFS                      |
 +--------------------------------*/

#if defined(MEN_Z77_USE_OWN_PHYACCESS)
/* Add more possible PHY IDs here */
static PHY_DEVICE_TBL z077PhyAttachTbl[] = {
	{ 0x0022, "Micrel "			},		/* 	Micrel PHY on EM01				*/
	{ 0x0141, "Marvell 88E6095"	},		/* 	Switch F301, F302				*/
	{ 0x0143, "Broadcom BCM5481"},		/* 	F11S 							*/
	{ 0x0013, "LXT971"			},		/*  P511+US03	 					*/
	{ 0x000d, "MEN PHY"			},		/*  dummy PHY in F218 rear Ethernet	*/
	{ 0xffff, ""}	/* end marker */
};
#endif

/**
 * z077_private: main data struct for the driver \n
 * this struct keeps all data for a per-IP-core instance of 16Z077/87
 */
/*@{*/
struct z77_private {
	long 			open_time;		/*!< how long is device open already */
    long	  		flags;			/*!< Our local flags 				 */
	u32  			instance;		/*!< chameleon instance if more Z87  */
	u32  			instCount;		/*!< global probe counter for phyadr[] */
	u32				nCurrTbd;		/*!< currently used Tx BD 			*/
	u32				txIrq;			/*!< last serviced TX IRQ			*/
	u32				modCode;		/*!< chameleon modCode				*/
  unsigned long		bdBase;			/*!< start address of BDs in RAM 	*/
	u32 			bdOff;			/*!< BDoffset from BAR start 		*/
	u32 			tbdOff;			/*!< TX Buffer offset to phys base  */
	u32 			rbdOff;			/*!< TX Buffer offset to phys base  */
	u32				serialnr;		/*!< serial nr. of P51x or MM1 		*/
	u32				board;			/*!< board identifier of this eth 	*/
	u32				mac_offset;		/*!< board specific MAC part		*/
	u32				gotBusy;		/*!< BUSY irq flag, clears in NAPI  */
#if defined( MEN_MM1 )
	SMB_DESC_PORTCB smb2desc;       /*!< SMB2 descriptor for EEPROM     */
	u32	 			osHdl;			/*!< OSS Handle 					*/
	void            *smbHdlP;		/*!< SMB2 Handle 					*/
#endif
	struct work_struct reset_task;  /*!< process context reseting (ndo_tx_timeout) */
#ifdef DO_NETDEV_TIMEOUT_TEST
	u32				nCount;			/*!< test counter */
	u32				doNDOtest;		/*!< test counter */
#endif
	struct timer_list timer;		/*!< period timer for linkchange poll */
	u32				timer_offset;	/*!< offset for link change poll */
	u32				prev_linkstate;	/*!< previous link state */
#if defined(Z77_USE_VLAN_TAGGING)
	struct vlan_group		*vlgrp; /*!< VLAN tagging group */
	u32				err_vlan;		/*!< VLAN error flags/count */
# define RXD_VLAN_MASK				0x0000ffff
# define VLAN_ETHER_TYPE 			0x8100
#endif
	u8					mcast_lst[MAX_MCAST_LST][MAC_ADDR_LEN]; /*!< store Mcast addrs */

	/* special: for P511 with nodma feature */
	u32 				ddr_len;		   	/*!< P511 PCI memory length */
	u32  				ddr_addr;		   	/*!< P511 PCI memory phys. address */
	void* 				ddr_virt;		   	/*!< P511 PCI memory virt. address */
	u32  				coreRev;			/*!< Z87 revision (1=with Rx count)	*/
	Z077_BD				txBd[Z077_TBD_NUM];	/*!< Tx Buffer Descriptor address */
	Z077_BD				rxBd[Z077_RBD_NUM]; /*!< Rx Buffer Descriptor address */
	struct net_device_stats stats;		/*!< status flags to report to IP */
	spinlock_t 			lock;			/*!< prevent concurrent accesses */
	struct pci_dev		*pdev;			/*!< the pci device we belong to */
#ifndef MEN_Z77_USE_OWN_PHYACCESS
	struct phy_device	*phy_dev;		/*!< phy device attached to us*/
	struct mii_bus		*mii_bus;		/*!< MDIO bus structure phy is attached */
	u32 				duplex;			/*!< duplexity (half or full) */
	u32					speed;			/*!< speed (10 or 100)	*/
#endif
	struct mii_if_info 	mii_if;			/*!< MII API hooks, info */
	u32 				msg_enable;		/*!< debug message level */
    #if LINUX_VERSION_CODE > VERSION_CODE(2,6,23)
	struct napi_struct 		napi;       /*!< NAPI struct */
    #endif
	struct net_device  		*dev;       /*!< net device */
};
/*@}*/

static int	z77_open(struct net_device *dev);
static int	z77_send_packet(struct sk_buff *skb, struct net_device *dev);
#if LINUX_VERSION_CODE >= VERSION_CODE(2,6,19)
static irqreturn_t z77_irq(int irq, void *dev_id);
#else
static irqreturn_t z77_irq(int irq, void *dev_id, struct pt_regs *regs);
#endif

static int	z77_close(struct net_device *dev);
static struct	net_device_stats *z77_get_stats(struct net_device *dev);
static void z77_tx_timeout(struct net_device *dev);
static void z77_rx_err(struct net_device *dev );
static void z77_tx_err(struct net_device *dev );
#if LINUX_VERSION_CODE > VERSION_CODE(2,6,23)
static int z77_poll(struct napi_struct *napi,int budget);
#else
static int z77_poll(struct net_device *dev,int *budget);
#endif
static int chipset_init(struct net_device *dev, unsigned int donegotiate);
#if defined(MEN_Z77_USE_OWN_PHYACCESS)
static int z77_phy_reset(struct net_device *dev, u8 phyAddr);
static int z77_phy_identify(struct net_device *dev, u8 phyAddr);
static int z77_phy_init(struct net_device *dev);
static int z77_init_phymode(struct net_device *dev, u8 phyAddr);
#else
static int z77_mii_bus_read( struct mii_bus *bus, int phy_id, int location);
static int z77_mii_bus_write( struct mii_bus *bus, int phy_id, int location, u16 val);
#endif
static int z77_pass_packet( struct net_device *dev, unsigned int idx );
static int z77_mdio_read(struct net_device *dev, int phy_id, int location);
static void z77_mdio_write(struct net_device *dev, int phy_id, int location, int val);
static void z77_reset( struct net_device *dev );
static int z77_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd);
static void z77_hash_table_setup(struct net_device *dev);
static int ether_gen_crc(struct net_device *dev, u8 *data);
#if defined(Z77_USE_VLAN_TAGGING)
# if LINUX_VERSION_CODE < VERSION_CODE(3,0,0) 
static void z77_vlan_rx_register(struct net_device *dev, struct vlan_group *grp);
static void z77_vlan_rx_kill_vid(struct net_device *dev, unsigned short vid);
# endif
static int z77_vlan_rx_add_vid(struct net_device *dev, unsigned short proto, unsigned short vid);
#endif

#if defined(MEN_Z77_USE_OWN_PHYACCESS)
static void z77_timerfunc(unsigned long);
#endif

/*--------------------------------+
 |  GLOBALS                       |
 +--------------------------------*/

/* filled depending on found IP core, either "16Z077" or "16Z087" */
static char cardname[16];
static const char *version = "$Id: men_16z077_eth.c,v 1.45 2014/07/16 19:30:45 ts Exp $";

/* helper for description of phy advertised/supported capabilities*/
static const char *phycaps[]={
	"10baseT_Half",	"10baseT_Full", "100baseT_Half", "100baseT_Full",
	"1000baseT_Half", "1000baseT_Full",	"Auto",	"TP", "AUI", "MII",	"FIBRE",
	"BNC", "10000baseT_Full", "Pause", "Asym_Pause"
};

/*
 * module parameters
 */
static char* mode 	 = "AUTO";
static int  nrcores  = NR_ETH_CORES_MAX;
static int  nodma    = 0;
static int  dbglvl   = 0;
static int  macadr   = 0;
static int  serialnr = 0;
#ifdef MEN_MM1
static int  writemac = 0x0;
#endif

/* global to count detected instances */
static int G_globalInstanceCount = 0;

/* array to tokenize / store multiple phy modes */
#define PHY_MODE_NAME_LEN	(5+1)
static char G_phyMode[NR_ETH_CORES_MAX][PHY_MODE_NAME_LEN];   /* sizeof("100FD\0"), longest string allowed */
static int  G_sernum=0;		/* serial number of CPU found in ID EEprom */

#ifdef CONFIG_MENEM9
int	phyadr[NR_ETH_CORES_MAX] = {0,1,2,3,4,5,6,7}; 	/* PHY addresses */
#else
int	phyadr[NR_ETH_CORES_MAX] = {0,0,0,0,0,0,0,0}; 	/* PHY addresses */
#endif /*CONFIG_MENEM9*/

module_param(mode, charp, 0 );
MODULE_PARM_DESC( mode, "PHY mode: AUTO,10HD,10FD,100HD,100FD. default: AUTO");
module_param_array( phyadr, int, (void*)&nrcores, 0664 );
MODULE_PARM_DESC( phyadr, "Address of PHY connected to each Z87 unit");
module_param( nodma, int, 0664 );
MODULE_PARM_DESC( nodma, "0: standard (DMA) function 1: P511 (no DMA) mode");
module_param( dbglvl, int, 0664 );
MODULE_PARM_DESC( dbglvl, "initial debug level. (0=none, 3=very verbose)");
module_param( macadr, int, 0664 );
MODULE_PARM_DESC( macadr, "forced MAC address to use");
#ifdef MEN_MM1
module_param( writemac, int, 0664 );
MODULE_PARM_DESC( writemac, "1: program MM1 serial Nr. as MAC. USE AT OWN RISK!");
module_param( sernum, int, 0664 );
MODULE_PARM_DESC( serialnr, "serial number of MM1 (USE AT OWN RISK!)");
#else
module_param( serialnr, int, 0664 );
MODULE_PARM_DESC( serialnr, "serial number of board");
#endif

/* helper to keep Register descriptions in a comfortable struct */
const Z077_REG_INFO z77_reginfo[] = {
	{"MODER     ", Z077_REG_MODER		},
	{"INT_SRC   ", Z077_REG_INT_SRC 	},
	{"INT_MASK  ", Z077_REG_INT_MASK 	},
	{"IPGT      ", Z077_REG_IPGT 		},
	{"IPGR1     ", Z077_REG_IPGR1 		},
	{"IPGR2     ", Z077_REG_IPGR2 		},
	{"PACKLEN   ", Z077_REG_PACKLEN 	},
	{"COLLCONF  ", Z077_REG_COLLCONF 	},
	{"TX_BDNUM  ", Z077_REG_TX_BDNUM 	},
	{"CTRLMODER ", Z077_REG_CTRLMODER 	},
	{"MIIMODER  ", Z077_REG_MIIMODER 	},
	{"MIICMD    ", Z077_REG_MIICMD 		},
	{"MIIADR    ", Z077_REG_MIIADR 		},
	{"MIITX_DATA", Z077_REG_MIITX_DATA 	},
	{"MIIRX_DATA", Z077_REG_MIIRX_DATA 	},
	{"MIISTATUS ", Z077_REG_MIISTATUS 	},
	{"MAC_ADDR0 ", Z077_REG_MAC_ADDR0 	},
	{"MAC_ADDR1 ", Z077_REG_MAC_ADDR1	},
	{"HASH_ADDR0", Z077_REG_HASH_ADDR0 	},
	{"HASH_ADDR1", Z077_REG_HASH_ADDR1 	},
	{"TXCTRL    ", Z077_REG_TXCTRL		},
	{"GLOBAL RST", Z077_REG_GLOBALRST	},
	{"BD_START  ", Z077_REG_BDSTART		},
	{"RX_EMPTY0 ", Z077_REG_RXEMPTY0	},
	{"RX_EMPTY1 ", Z077_REG_RXEMPTY1	},
	{"TX_EMPTY0 ", Z077_REG_TXEMPTY0	},
	{"TX_EMPTY1 ", Z077_REG_TXEMPTY1	},
	{"RX_BDSTAT ", Z077_REG_RXBDSTAT	},
	{"SMBCTRL   ", Z077_REG_SMBCTRL  	},
	{"COREREV   ", Z077_REG_COREREV  	},
	{"RXERRCNT1 ", Z077_REG_RXERRCNT1	},
	{"RXERRCNT2 ", Z077_REG_RXERRCNT2	},
	{"TXERRCNT1 ", Z077_REG_TXERRCNT1	},
	{"TXERRCNT2 ", Z077_REG_TXERRCNT2	},
	{NULL, 0xffff} /* end mark */
};

/*****************************************************************************/
/** Handle multicast and promiscuous mode set.
 *
 *  The set_multi entry point is called whenever the multicast address
 *  list or the network interface flags are updated.  This routine is
 *  responsible for configuring the hardware for proper multicast,
 *  promiscuous mode, and all-multi behavior.
 *
 * reminder for the flags:
 * IFF_UP          0x1                interface is up
 * IFF_BROADCAST   0x2                broadcast address valid
 * IFF_DEBUG       0x4                turn on debugging
 * IFF_LOOPBACK    0x8                is a loopback net
 * IFF_POINTOPOINT 0x10               interface is has p-p link
 * IFF_NOTRAILERS  0x20               avoid use of trailers
 * IFF_RUNNING     0x40               interface RFC2863 OPER_UP
 * IFF_NOARP       0x80               no ARP protocol
 * IFF_PROMISC     0x100              receive all packets
 * IFF_ALLMULTI    0x200              receive all multicast packets
 * IFF_MASTER      0x400              master of a load balancer
 * IFF_SLAVE       0x800              slave of a load balancer
 * IFF_MULTICAST   0x1000             Supports multicast
 * IFF_PORTSEL     0x2000             can set media type
 * IFF_AUTOMEDIA   0x4000             auto media select active
 * IFF_DYNAMIC     0x8000             dialup device with changing addresses
 * IFF_LOWER_UP    0x10000            driver signals L1 up
 * IFF_DORMANT     0x20000            driver signals dormant
 * IFF_ECHO        0x40000            echo sent packets
 *
 * \param dev		\IN net_device struct for this NIC
 */
static void z77_set_rx_mode(struct net_device *dev)
{
	struct z77_private *np = netdev_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&np->lock, flags);

	Z77DBG( ETHT_MESSAGE_LVL1, MEN_Z77_DRV_NAME " z77_set_rx_mode: dev %s flags:0x%x\n",
			dev->name, dev->flags);
	/*
	 * check independently if current flag setting and requested flags differ. Otherwise
	 * _PRO is set every time _IAM changes and vice versa.
	 */
	if ( dev->flags & IFF_PROMISC) {
		if ( !(Z77READ_D32(Z077_BASE, Z077_REG_MODER) & OETH_MODER_PRO)) {
			/* promisc was cleared, set it */
			Z77DBG( ETHT_MESSAGE_LVL2, MEN_Z77_DRV_NAME " dev %s: set IFF_PROMISC\n", dev->name);
			Z077_SET_MODE_FLAG(OETH_MODER_PRO);
		}
	} else {
		if ( (Z77READ_D32(Z077_BASE, Z077_REG_MODER) & OETH_MODER_PRO)) {
			/* promisc was set, clear it */
			Z77DBG( ETHT_MESSAGE_LVL2, MEN_Z77_DRV_NAME " dev %s clear IFF_PROMISC\n", dev->name);
			Z077_CLR_MODE_FLAG(OETH_MODER_PRO);
		}
	}

	if ( dev->flags & IFF_MULTICAST) {
		if ( !(Z77READ_D32(Z077_BASE, Z077_REG_MODER) & OETH_MODER_IAM)) {
			/* mcast hash usage was cleared, set it */
			Z77DBG( ETHT_MESSAGE_LVL2, MEN_Z77_DRV_NAME " dev %s: set IFF_MULTICAST\n", dev->name);
			Z077_SET_MODE_FLAG(OETH_MODER_IAM);
		}
		/* update the HASH0/1 bits in any case, could also be a removed mcast MAC */
		z77_hash_table_setup( dev );
	} else {
		if ( (Z77READ_D32(Z077_BASE, Z077_REG_MODER) & OETH_MODER_IAM)) {
			/* mcast hash usage was cleared, set it and add passed MAC to mc_list */
			Z77DBG( ETHT_MESSAGE_LVL2, MEN_Z77_DRV_NAME " dev %s clear IFF_MULTICAST\n", dev->name);
			Z077_CLR_MODE_FLAG(OETH_MODER_IAM);
		}
	}

	spin_unlock_irqrestore (&np->lock, flags);
	return;
}

/******************************************************************************
 ** calculate multicast hash bin value
 *
 * \return	hash bit position (0-63) on success or -1 on Error
 */
static int ether_gen_crc(struct net_device *dev, u8 *data)
{
	u32 crc 	= MCAST_HASH_CRC_SEED;
	int length 	= MAC_ADDR_LEN;
	int hashbin = 0;
	u8 curr_oct = 0;
	u8 *p 		= data;
	int bit		= 0;
	struct z77_private *np = netdev_priv(dev);

	if (data == NULL)
		return -1;

	Z77DBG( ETHT_MESSAGE_LVL2,
			MEN_Z77_DRV_NAME " ether_gen_crc: Address= %02x:%02x:%02x:%02x:%02x:%02x - ",
			p[0], p[1], p[2], p[3], p[4], p[5] );

	while ( --length >= 0 ) {
		curr_oct = *data++;
		for (bit = 0; bit < 8; bit++ ) {
			if ( ((crc & 0x80000000) ? 1 : 0) != (curr_oct & 0x1) ) {
				crc = (crc << 1) ^ MCAST_HASH_POLYNOM;
			} else {
				crc <<=1;
			}
			crc = crc & 0xffffffff;
			curr_oct >>=1;
		}
	}

	/* return bin position */
	hashbin = (int)((crc >> MCAST_MULT_SHFT) & MCAST_HASH_MASK);
	Z77DBG( ETHT_MESSAGE_LVL2, MEN_Z77_DRV_NAME " bin=0x%02x\n", hashbin );
	return(hashbin);

}  /* ether_gen_crc */


/******************************************************************************
 ** setup hash table and bits in HASH0/1 when a multicast MAC is set up
 *  or removed
 *
 * \return	0 on success or -EINVAL
 */
static void z77_hash_table_setup(struct net_device *dev)
{
	int i = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)	
	struct netdev_hw_addr *ha = NULL;
#else
	struct dev_mc_list *ptr=NULL;
#endif

	u8 *p=NULL;
	struct z77_private *np = netdev_priv(dev);
	u32 hash0 = 0;
	u32 hash1 = 0;
	u32 bin_pos = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
	Z77DBG( ETHT_MESSAGE_LVL2, MEN_Z77_DRV_NAME " z77_hash_table_setup: mc_count = %d\n",
			netdev_mc_count(dev));
	if ((dev->flags & IFF_MULTICAST) && netdev_mc_count(dev)) {
		if (netdev_mc_count(dev) <= MAX_MCAST_LST) {
			netdev_for_each_mc_addr(ha, dev) {
				/* check if its a valid MC addr: bit1 in mac[0] set ? */
				if ((ha->addr[0] & 0x1) == 0x0)
					continue;
				memcpy(np->mcast_lst[i], ha->addr, MAC_ADDR_LEN);
				p = (u8*)(np->mcast_lst[i]);

				/* collect every hash bit, OR it together and update HASH0 and HASH1 registers */
				bin_pos = ether_gen_crc(dev, p );

				if (bin_pos > 31)
					hash1 |=( 1<<(bin_pos-32));
				else
					hash0 |=( 1<<bin_pos);
				i++;
			}
		}
	}
#else
	Z77DBG( ETHT_MESSAGE_LVL2, MEN_Z77_DRV_NAME " z77_hash_table_setup: mc_count = %d\n",
			dev->mc_count);
	if ((dev->flags & IFF_MULTICAST) && dev->mc_count) {
		if (dev->mc_count <= MAX_MCAST_LST) {
			for ( ptr = dev->mc_list; ptr ; ptr = ptr->next ) {
				/* check if its a valid MC addr: bit1 in mac[0] set ? */
				if (!(*ptr->dmi_addr & 1))
					continue;

				memcpy(np->mcast_lst[i], ptr->dmi_addr, MAC_ADDR_LEN);
				p = (u8*)(np->mcast_lst[i]);

				/* collect every hash bit, OR it together and update HASH0 and HASH1 registers */
				bin_pos = ether_gen_crc(dev, p );

				if (bin_pos > 31)
					hash1 |=( 1<<(bin_pos-32));
				else
					hash0 |=( 1<<bin_pos);
				i++;
			}
		}
	}
#endif

	Z77DBG( ETHT_MESSAGE_LVL2,
			MEN_Z77_DRV_NAME " z77_hash_table_setup: HASH0=0x%08x HASH1=0x%08x\n",
			hash0, hash1);

	Z77WRITE_D32( Z077_BASE, Z077_REG_HASH_ADDR0, hash0 );
	Z77WRITE_D32( Z077_BASE, Z077_REG_HASH_ADDR1, hash1 );

	return;
}

/* new API: net_device_ops moved out of struct net_device in own ops struct */
#if LINUX_VERSION_CODE > VERSION_CODE(2,6,30)

/******************************************************************************
 ** set new MAC address: unused here
 *
 * \return	0 always
 */
static int z77_set_mac_address(struct net_device *dev, void *p)
{
	struct z77_private *np = netdev_priv(dev);
	struct sockaddr *addr = p;
	unsigned long flags;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

	/* shut-down */
	z77_reset( dev );

	/* set MAC Address in MAC_ADDR0, MAC_ADDR1 */
	spin_lock_irqsave(&np->lock, flags);

	Z77WRITE_D32( Z077_BASE, Z077_REG_MAC_ADDR0,
				  dev->dev_addr[2] << 24 | dev->dev_addr[3] << 16 | \
				  dev->dev_addr[4] << 8  | dev->dev_addr[5]);
	Z77WRITE_D32( Z077_BASE, Z077_REG_MAC_ADDR1,
				  dev->dev_addr[0] << 8 | dev->dev_addr[1] );

	/* force link-up */
	np->prev_linkstate = 0;

	spin_unlock_irqrestore (&np->lock, flags);
	return 0;
}




/******************************************************************************
 ** set new MTU for the interface
 *
 * \return	0 on success or -EINVAL
 */
static int z77_change_mtu(struct net_device *netdev, int new_mtu)
{
	if (new_mtu < ETH_ZLEN || new_mtu > ETH_DATA_LEN)
		return -EINVAL;
	netdev->mtu = new_mtu;
	return 0;
}

static const struct net_device_ops z77_netdev_ops = {
	.ndo_open				= z77_open,
	.ndo_stop				= z77_close,
	.ndo_start_xmit			= z77_send_packet,
	.ndo_do_ioctl			= z77_ioctl,
	.ndo_get_stats			= z77_get_stats,
	.ndo_tx_timeout			= z77_tx_timeout,
	.ndo_set_rx_mode 		= z77_set_rx_mode,
	.ndo_change_mtu			= z77_change_mtu,
	.ndo_validate_addr		= eth_validate_addr,
	.ndo_set_mac_address	= z77_set_mac_address,
#if defined(Z77_USE_VLAN_TAGGING) && defined(Z77_VLAN_OPS_CHANGE)
#if LINUX_VERSION_CODE < VERSION_CODE(3,0,0) 
	.ndo_vlan_rx_register	= z77_vlan_rx_register,
	.ndo_vlan_rx_kill_vid   = z77_vlan_rx_kill_vid,
#endif
	.ndo_vlan_rx_add_vid	= z77_vlan_rx_add_vid,
#endif

};
#endif


#ifdef MEN_MM1
/******************************************************************************
 ** z77_scl_in - read SCL Pin on SMB Register
 *
 * \param dat	\IN general purpose data, the Z87 instances base address
 *
 * \return		pin state: 0 or 1
 */
static int z77_scl_in(void *dat)
{
	int pin=0;
	volatile unsigned int tmp = Z77READ_D32( dat, Z077_REG_SMBCTRL);
	pin = tmp & SMB_REG_SCL ? 1 : 0;
	return pin;
}

/******************************************************************************
 ** z77_sda_in - read SDA Pin on SMB Register
 *
 * \param dat	\IN general purpose data, the Z87 instances base address
 *
 * \return		pin state: 0 or 1
 */
static int z77_sda_in(void *dat)
{
	int pin=0;
	volatile unsigned int tmp = 0, i = 0;

	for (i=0; i < 2; i++) {
		tmp = Z77READ_D32( dat, Z077_REG_SMBCTRL);
	}

	pin = (tmp & SMB_REG_SDA) ? 1 : 0;
	return pin;
}

/******************************************************************************
 ** z77_sda_out - set SDA Pin on SMB Register
 *
 * \param dat	 \IN general purpose data, the Z87 instances base address
 * \param pinval \IN pin state to set, 0 or 1
 *
 * \return		 0 if no difference or 1 if pin state not equal to pinval
 */
static int z77_sda_out(void *dat, int pinval)
{

	volatile unsigned int tmp 	= 	Z77READ_D32( dat, Z077_REG_SMBCTRL);

	if (pinval)
		tmp |=SMB_REG_SDA;
	else
		tmp &=~SMB_REG_SDA;

	Z77WRITE_D32( dat, Z077_REG_SMBCTRL, tmp);

	return(0);
}

/******************************************************************************
 ** z77_scl_out - set SCL Pin on SMB Register
 *
 * \param dat	 \IN general purpose data, the Z87 instances base address
 * \param pinval \IN pin state, 0 or 1
 *
 * \return		 0 if no difference or 1 if pin state not equal to pinval
 */
static int z77_scl_out(void *dat, int pinval)
{

	volatile unsigned int tmp 		= 	Z77READ_D32( dat, Z077_REG_SMBCTRL);
	if (pinval)
		tmp |=SMB_REG_SCL;
	else
		tmp &=~SMB_REG_SCL;
	Z77WRITE_D32( dat, Z077_REG_SMBCTRL, tmp );
	return(0);
}
#endif


#if defined (MEN_Z77_USE_OWN_PHYACCESS)
/******************************************************************************
 ** z77_timerfunc - periodically check the link state
 *
 * \param dat	\IN general purpose data, used as z77_private struct
 *
 * \return			-
 */
static void z77_timerfunc(unsigned long dat)
{
	u32 linkstate;
	struct net_device *dev = (struct net_device *)dat;
	struct z77_private *np = netdev_priv(dev);

	linkstate = mii_link_ok(&np->mii_if);
	if ( np->prev_linkstate != linkstate ) {
		if (linkstate == 1) { /* link came up: restart IP core */
			z77_reset( dev );
			Z077_SET_MODE_FLAG( OETH_MODER_RXEN | OETH_MODER_TXEN );
			np->nCurrTbd = 0;
			printk( MEN_Z77_DRV_NAME " (%s): link is up\n", dev->name);
		} else {/* link went down: close device */
			printk( MEN_Z77_DRV_NAME " (%s): link is down\n", dev->name);
		}
		np->prev_linkstate = linkstate;
	}

	/* restart timer */
	np->timer.expires = jiffies + np->timer_offset;
 	add_timer(&np->timer);
}
#endif /* defined (MEN_Z77_USE_OWN_PHYACCESS) */


/******************************************************************************
 ** z77_regdump - Dump the contents of the 16Z077/087 Registers and BDs
 *
 * \param dev		\IN net_device struct for this NIC
 *
 * \return			-
 */
static int z77_regdump( struct net_device *dev )
{
	u32 adr	= 0;
	u32 tmp = 0;
#ifndef NO_PHY
 	u16 dat = 0;
#endif
  unsigned long i = 0;
	struct z77_private *np = netdev_priv(dev);

	printk( KERN_INFO MEN_Z77_DRV_NAME " (netdevice '%s') base Addr: 0x%08lx\n",
			dev->name, dev->base_addr );

	printk( KERN_INFO "np->bdOff    0x%04x\n", np->bdOff);
	printk( KERN_INFO "np->tbdOff   0x%04x\n", np->tbdOff);
	printk( KERN_INFO "np->rbdOff   0x%04x\n", np->rbdOff);

	while (z77_reginfo[i].addr != 0xffff){
		tmp = Z77READ_D32( dev->base_addr, z77_reginfo[i].addr);
		printk( KERN_INFO "%s   0x%08x\n", z77_reginfo[i].name, tmp);
		i++;
	}

	printk( KERN_INFO "current TXBd 0x%02x\n", np->nCurrTbd);

#ifndef NO_PHY
	printk( KERN_INFO "----- MII Registers -----\n");
	printk( KERN_INFO "instance\t\t\t0x%02x\n", np->instance );
	printk( KERN_INFO "PHY ADR\t\t\t0x%02x\n", 	np->mii_if.phy_id );

	dat = z77_mdio_read(dev, np->mii_if.phy_id, MII_BMCR );
	printk( KERN_INFO "MII_BMCR\t\t\t0x%04x\n", dat );

	/* ts: initially the link status isnt up to date, read it twice.. */
	dat = z77_mdio_read(dev, np->mii_if.phy_id , MII_BMSR );
	dat = z77_mdio_read(dev, np->mii_if.phy_id , MII_BMSR );

	printk( KERN_INFO "MII_BMSR\t\t0x%04x\tLink: %s\n",
		   dat, (dat & BMSR_LSTATUS) ? "up" : "down" );

	dat = z77_mdio_read(dev, np->mii_if.phy_id , MII_PHYSID1 );
	printk( KERN_INFO "MII_PHYSID1\t\t0x%04x\n", dat );

	dat = z77_mdio_read(dev, np->mii_if.phy_id , MII_PHYSID2 );
	printk( KERN_INFO "MII_PHYSID2\t\t0x%04x\n", dat );

	dat = z77_mdio_read(dev, np->mii_if.phy_id , MII_ADVERTISE );
	printk( KERN_INFO "MII_ADVERTISE\t\t0x%04x\n", dat );

	dat = z77_mdio_read(dev, np->mii_if.phy_id , MII_LPA );
	printk( KERN_INFO "MII_LPA\t\t\t0x%04x\n", dat );

	dat = z77_mdio_read(dev, np->mii_if.phy_id , MII_100_BASE_TX_PHY);

	printk( KERN_INFO "MII_100_BASE_TX_PHY\t0x%04x (", dat );
	/* dump PHY Op Mode Indication Reg. Bits [4..2] */

	switch((dat & 0x1c) >> 2) {
	case 0x0:
		printk( KERN_INFO "in Autoneg)\n");
		break;
	case 0x1:
		printk( KERN_INFO "10MB HD)\n");
		break;
	case 0x2:
		printk( KERN_INFO "100MB HD)\n");
		break;
	case 0x3:
		printk( KERN_INFO "Default)\n");
		break;
	case 0x5:
		printk( KERN_INFO "10MB FD)\n");
		break;
	case 0x6:
		printk( KERN_INFO "100MB FD)\n");
		break;
	case 0x7:
		printk( KERN_INFO "PHY Isolated)\n");
		break;
	default:
		printk( KERN_INFO "unknown)\n");
		break;
	}
#endif
	if (np->msg_enable >= ETHT_MESSAGE_LVL2) {
		printk(KERN_INFO "------------------ TX BDs: -------------------\n");
		for (i = 0; i < Z077_TBD_NUM; i++ ) {
			adr = Z077_BD_OFFS + (i * Z077_BDSIZE);
			printk(KERN_INFO "%02x STAT: 0x%04x LEN: 0x%04x  ADR 0x%08x\n", i,
	     Z077_GET_TBD_FLAG(i, 0xffff), 
	     Z077_GET_TBD_LEN(i),
				   Z077_GET_TBD_ADDR(i));
		}

		printk(KERN_INFO "------------------ RX BDs: -------------------\n");
		for (i = 0; i < Z077_RBD_NUM ; i++ ) {
			adr = Z077_BD_OFFS + (i + Z077_TBD_NUM ) * Z077_BDSIZE;
			printk(KERN_INFO "%02x STAT: 0x%04x LEN: 0x%04x  ADR 0x%08x\n", i,
				   Z077_GET_RBD_FLAG(i, 0xffff), Z077_GET_RBD_LEN(i),
				   Z077_GET_RBD_ADDR(i) );
		}
	}
	return 0;
}



/******************************************************************************
 ** z77_mdio_read  Wrapper for MII read access
 *
 * \param dev		\IN net_device struct for this NIC
 * \param phy_id	\IN address of the PHY
 * \param location	\IN net_device struct for this NIC
 *
 * \return			Value read from PHY <phy_id> register <location>
 */
static int z77_mdio_read(struct net_device *dev, int phy_id, int location)
{

	int retVal 			= 0xffff;
	volatile u32 miival = 0;
	volatile u32 tout 	= MII_ACCESS_TIMEOUT;

	/* wait until a previous BUSY disappears */
	do {
		miival = Z77READ_D32(Z077_BASE, Z077_REG_MIISTATUS );
		tout--;
	} while( (miival & OETH_MIISTATUS_BUSY) && tout);

	if (!tout) {
		printk(KERN_ERR "*** MII Read timeout!\n");
		return -1;
	}

	/* set up combined PHY and Register within Phy */
	Z77WRITE_D32( Z077_BASE, Z077_REG_MIIADR, (location & 0xff) << 8 | phy_id );

	/* kickoff Read Command, this asserts BUSY*/
	Z77WRITE_D32( Z077_BASE, Z077_REG_MIICMD, OETH_MIICMD_RSTAT);

	/* wait until the PHY finished */
	do {
		miival = Z77READ_D32(Z077_BASE, Z077_REG_MIISTATUS );
		tout--;
	} while( (miival & OETH_MIISTATUS_BUSY) && tout);

	if (!tout) {
		printk(KERN_ERR "*** MII Read timeout!\n");
		return -1;
	}

    /* fetch read Value from MIIRX_DATA*/
    retVal = Z77READ_D32( Z077_BASE, Z077_REG_MIIRX_DATA );
	return retVal;
}

#if !defined(MEN_Z77_USE_OWN_PHYACCESS)
/******************************************************************************
 ** z77_mii_bus_read	Wrapper for MII read access with phy access functions
 *
 * \param bus		\IN mii_bus struct for this NIC
 * \param phy_id	\IN address of the PHY
 * \param location	\IN net_device struct for this NIC
 *
 * \return			0 or -ENODEV if bus or net_device not initialized
 */
static int z77_mii_bus_read( struct mii_bus *bus, int phy_id, int location)
{
	if (bus) {
		struct net_device *dev = (struct net_device *)bus->priv;
		if (dev)
			return z77_mdio_read( dev, phy_id, location );
		else
			return -ENODEV;
	}
	else
		return -ENODEV;
}

/******************************************************************************
 ** z77_mii_bus_write  	Wrapper for MII write access with phy access functions
 *
 * \param bus		\IN mii_bus struct for this NIC
 * \param phy_id	\IN address of the PHY
 * \param location	\IN net_device struct for this NIC
 * \param val 		\IN value to write to MII Register
 *
 * \return			0 or -ENODEV if bus or net_device not initialized
 */
static int z77_mii_bus_write( struct mii_bus *bus, int phy_id,
							   int location, u16 val)
{
	if (bus) {
		struct net_device *dev = (struct net_device *)bus->priv;
		if (dev) {
			z77_mdio_write( dev, phy_id, location, val );
			return 0;
		} else
			return -ENODEV;
	} else
		return -ENODEV;
}
#endif /* !defined(MEN_Z77_USE_OWN_PHYACCESS) */


/******************************************************************************
 ** z77_parse_mode    simple strtok replacement to split up mode string.
 *                    delimiter is ',' as with all module array arguments.
 * \return			0 on success or -1 on error
 */
static int z77_parse_mode(int len, char *pChar)
{
	unsigned int arglen = 0;
	int i,j=0,k=0;

	if ( (pChar == NULL) || (len == 0))
		return -1;

	arglen = len;
	memset(G_phyMode, 0x0, sizeof(G_phyMode));

	/* the mode string can be max. [NR_ETH_CORES_MAX * "100FD," minus last comma */
	if ( arglen > ( (NR_ETH_CORES_MAX * (PHY_MODE_NAME_LEN+1))-1))
		arglen = ( NR_ETH_CORES_MAX * (PHY_MODE_NAME_LEN+1) )-1;

	for ( i=0; i < arglen; i++) {
		if (pChar[i] != ',') {
			G_phyMode[j][k] = pChar[i];
			/* maximum length per single mode reached ? */
			if (k < (PHY_MODE_NAME_LEN-1))
				k++;
			else
				G_phyMode[j][k] = '\0';
		} else {
			j++;
			k=0;
		}
	}

	return j;
}


/******************************************************************************
 ** z77_mdio_write  	Wrapper for MII write access
 *
 * \param dev		\IN net_device struct for this NIC
 * \param phy_id	\IN address of the PHY (= 1 on EM01)
 * \param location	\IN net_device struct for this NIC
 * \param val 		\IN value to write to MII Register
 *
 * \return			-
 */
static void z77_mdio_write(struct net_device *dev, int phy_id,
						   int location, int val)
{

	volatile u32 miival = 0;
	volatile u32 tout 	= MII_ACCESS_TIMEOUT;

	/* wait until a previous BUSY disappears */
	do {
		miival = Z77READ_D32(Z077_BASE, Z077_REG_MIISTATUS );
		tout--;
	} while( (miival & OETH_MIISTATUS_BUSY) && tout);

	if (!tout) {
		printk(KERN_ERR "*** MII Write timeout!\n");
		return;
	}

	Z77WRITE_D32( Z077_BASE, Z077_REG_MIIADR, (location & 0xff) << 8 | phy_id );
	Z77WRITE_D32( Z077_BASE, Z077_REG_MIITX_DATA, val );
	Z77WRITE_D32( Z077_BASE, Z077_REG_MIICMD,  OETH_MIICMD_WCTRLDATA );

	/* wait until a previous BUSY disappears */
	do {
		miival = Z77READ_D32(Z077_BASE, Z077_REG_MIISTATUS );
		tout--;
	} while( (miival & OETH_MIISTATUS_BUSY) && tout);

	if (!tout) {
		printk(KERN_ERR "*** MII Write timeout!\n");
		return;
	}
}


/**
 * \defgroup _NETOPS_FUNC ethtool support functions
 */
/*@{*/
/******************************************************************************
 ** z77_ethtool_get_drvinfo retrieve drivers info with ethtool
 *
 * \param dev		\IN net_device struct for this NIC
 * \param info		\IN ethtool supporting info struct
 *
 * \return			-
 */
static void z77_ethtool_get_drvinfo(struct net_device *dev,
								struct ethtool_drvinfo *info)
{
	struct z77_private  *np = netdev_priv(dev);
#ifdef CONFIG_PCI
	struct pci_dev		*pcd = np->pdev;
#endif
	unsigned long flags;

	spin_lock_irqsave(&np->lock, flags);

	strncpy(info->driver, cardname, sizeof(info->driver)-1);
	strncpy(info->version, version, sizeof(info->version)-1);
#ifdef CONFIG_PCI
	if (pcd)
		strcpy(info->bus_info, pci_name(pcd));
#endif

	/* ts: added Register Dumps */
	if (np->msg_enable)
		z77_regdump(dev);

	spin_unlock_irq(&np->lock);

}



/******************************************************************************
 ** z77_dump_ecmd helper function to dump contents of ethtool command struct
 *
 * \param ecmd		\IN command type passed to ethtool, see linux/ethtool.h
 *
 * \return			0;
 */
static void z77_dump_ecmd(struct ethtool_cmd *ecmd)
{

	unsigned int i = 0;
	printk( KERN_INFO "main contents of ethtool_cmd struct:\n");
	printk( KERN_INFO "cmd          0x%08x\n", ecmd->cmd );

	printk( KERN_INFO "supported    0x%08x\n", ecmd->supported );
	for (i = 0; i < 15; i++)
		if ( (1<<i) & ecmd->supported)
			printk("    + %s\n", phycaps[i]);
	printk( KERN_INFO "advertising  0x%08x\n", ecmd->advertising );
	for (i = 0; i < 15; i++)
		if ( (1<<i) & ecmd->advertising)
			printk("    + %s\n", phycaps[i]);

	printk( KERN_INFO "speed        %d\n", ecmd->speed );
	printk( KERN_INFO "duplex       %d (%s)\n",
			ecmd->duplex, ecmd->duplex ? "full" : "half" );
	/*
	 * Enable or disable autonegotiation.  If this is set to enable,
	 * the forced link modes above are completely ignored.
	 */
	printk( KERN_INFO "autoneg      %d (%s)\n",
			ecmd->autoneg, ecmd->autoneg ? "on" : "off" );

}



/******************************************************************************
 ** z77_ethtool_get_settings retrieve NIC settings with ethtool
 *
 * \param dev		\IN net_device struct for this NIC
 * \param ecmd		\IN command type passed to ethtool, see linux/ethtool.h
 *
 * \return			0;
 */
static int z77_ethtool_get_settings(struct net_device *dev,
									struct ethtool_cmd *ecmd)
{
	struct z77_private *np = netdev_priv(dev);
	unsigned long flags;

	if( !(np->flags & IFF_UP) ) {
	     return -ENETDOWN;
	}

	spin_lock_irqsave(&np->lock, flags);
	mii_ethtool_gset(&np->mii_if, ecmd);

#ifdef NO_PHY
	/* get duplex setting from z087 register */
	if (Z77READ_D32(Z077_BASE, Z077_REG_MODER)&(OETH_MODER_FULLD)) {
		ecmd->duplex = DUPLEX_FULL;
	} else {
		ecmd->duplex = DUPLEX_HALF;
	}
	Z77DBG( ETHT_MESSAGE_LVL1,"%s: could only get duplex mode\n", __FUNCTION__);
#endif

	if (np->msg_enable == ETHT_MESSAGE_LVL3)
		z77_dump_ecmd(ecmd);

	spin_unlock_irqrestore (&np->lock, flags);
	return 0;
}


/******************************************************************************
 ** z77_ethtool_set_settings set NIC settings with ethtool
 *
 * \param dev		\IN net_device struct for this NIC
 * \param ecmd		\IN command type passed to ethtool, see linux/ethtool.h
 *
 * \return			0 or negative error number;
 */
static int z77_ethtool_set_settings(struct net_device *dev,
									struct ethtool_cmd *ecmd)
{
#ifndef NO_PHY
	struct ethtool_cmd ncmd;
#endif
	struct z77_private *np = netdev_priv(dev);
	int res=0;
	unsigned long flags;

	if( !(np->flags & IFF_UP) ) {
		return -ENETDOWN;
	}

	spin_lock_irqsave(&np->lock, flags);

	if (np->msg_enable == ETHT_MESSAGE_LVL3)
		z77_dump_ecmd(ecmd);

	if (ecmd->cmd != ETHTOOL_TEST) {
		res = mii_ethtool_sset(&np->mii_if, ecmd);
		/* wait to let settings take effect */

#if LINUX_VERSION_CODE > VERSION_CODE(2,6,30)
		/* set_current_state(TASK_INTERRUPTIBLE); */
		schedule_timeout_interruptible(CONFIG_HZ/4);
#else
		current->state = TASK_INTERRUPTIBLE;
		schedule_timeout(CONFIG_HZ/4);
#endif

#ifndef NO_PHY
		/* check PHY again, set MODER[10] to match duplexity setting in it */
		mii_ethtool_gset(&np->mii_if, &ncmd);

		/* hand over duplexity from phy */
		if (ncmd.duplex == DUPLEX_FULL) {
			Z077_SET_MODE_FLAG(OETH_MODER_FULLD);
		} else {
			Z077_CLR_MODE_FLAG(OETH_MODER_FULLD);
		}
#else
		/* set duplexity as given from ethtool! */
		if (ecmd->duplex == DUPLEX_FULL)
			Z077_SET_MODE_FLAG(OETH_MODER_FULLD);
		else
			Z077_CLR_MODE_FLAG(OETH_MODER_FULLD);
#endif

	} else {
		printk (KERN_INFO "activating compliance pattern test\n");
		/* start compliance pattern mode */

	}
	spin_unlock_irqrestore (&np->lock, flags);

	/* force link-up */
	np->prev_linkstate = 0;

	return res;
}

/******************************************************************************
 ** z77_ethtool_nway_reset restart MII
 *
 * \param dev		\IN net_device struct for this NIC
 *
 * \return			0 or error value;
 */
static int z77_ethtool_nway_reset(struct net_device *dev)
{
	struct z77_private *np = netdev_priv(dev);
	return mii_nway_restart(&np->mii_if);
}


/******************************************************************************
 ** z77_ethtool_get_link
 * check link status
 *
 * \param dev		\IN net_device struct for this NIC
 *
 * \return			1 if link us up or 0
 */
static u32 z77_ethtool_get_link(struct net_device *dev)
{
	struct z77_private *np = netdev_priv(dev);
	return mii_link_ok(&np->mii_if);
}

/******************************************************************************
 ** z77_ethtool_get_msglevel - get message level
 *
 *
 * \param dev		\IN net_device struct for this NIC
 *
 * \return			message verbosity level
 */
static u32 z77_ethtool_get_msglevel(struct net_device *dev)
{
	struct z77_private *np = netdev_priv(dev);
	/* printk("z77_ethtool_get_msglevel: v = %d\n", np->msg_enable ); */
	return np->msg_enable;
}

/******************************************************************************
 ** z77_ethtool_set_msglevel - Set message verbosity level
 *
 * \param dev	\IN net_device struct for this NIC
 * \param v		\IN message verbosity. 0 disable 1 normal 2 verbose 3 + IRQs
 * \brief		this is called when user calls ethtool -s eth0 msglvl <v>
 *
 * \return	-
 */
static void z77_ethtool_set_msglevel(struct net_device *dev, u32 v)
{
	struct z77_private *np = netdev_priv(dev);
	if (v > 3)
		v = 3;
	np->msg_enable = v;
}

/******************************************************************************
 ** z77_ethtool_testmode - Set PHY test mode 
 *
 * \param dev	\IN net_device struct for this NIC
 * \param etest	\IN test function(s), unused
 * \param data	\IN result data
 * \brief		this is called when user calls ethtool -t eth0 msglvl
 *
 * \return	-
 */
static void z77_ethtool_testmode(struct net_device *dev, struct ethtool_test *etest, u64 *data)
{
	printk("TODO: setting %s into test pattern mode\n", dev->name);
	/* TODO: yet to come */
	return;
}
/*@}*/   /* end defgroup ethtool funcs */



/**
 * This structure provides the interface functions to the standard ethtool
 */
static struct ethtool_ops z77_ethtool_ops = {
	.get_drvinfo 	= z77_ethtool_get_drvinfo,
	.get_settings 	= z77_ethtool_get_settings,
	.set_settings 	= z77_ethtool_set_settings,
	.nway_reset 	= z77_ethtool_nway_reset,
	.get_link 	= z77_ethtool_get_link,
	.get_msglevel 	= z77_ethtool_get_msglevel,
	.set_msglevel 	= z77_ethtool_set_msglevel,
	.self_test	= z77_ethtool_testmode,
};


/* return non zero if the Tx BD is full already, a stall condition
 occured */
u32 tx_full(struct net_device *dev)
{
	int txbEmpty;
	struct z77_private *np = netdev_priv(dev);

	if ( np->modCode == CHAMELEON_16Z077_ETH ) {
		txbEmpty = Z077_GET_TBD_FLAG(np->nCurrTbd, OETH_TX_BD_READY);
	} else {
		/* Z87 Core with extra TXBd empty Flags */
		if ( np->nCurrTbd < 32 )
			txbEmpty = Z77READ_D32(Z077_BASE, Z077_REG_TXEMPTY0)
					   & (1 << np->nCurrTbd);
		else
			txbEmpty = Z77READ_D32(Z077_BASE, Z077_REG_TXEMPTY1)
					   & (1 << (np->nCurrTbd-32));
	}

	return !txbEmpty;
}


#if ( LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,21) )
/* ts@men: we need the true linkstate for F218R01-01 */
static ssize_t z77_show_linkstate(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int bsmr = 0;
	struct z77_private *np  = netdev_priv(to_net_dev(dev));
#define  MII_BMSR_LINK_VALID	0x0004
	/* ts: initially the link status isnt up to date, read it twice. This is often
	   a latched Register inside PHYs */
	bsmr = z77_mdio_read(np->dev, np->mii_if.phy_id , MII_BMSR );
	bsmr = z77_mdio_read(np->dev, np->mii_if.phy_id , MII_BMSR );
	return sprintf(buf, "%c\n", bsmr & MII_BMSR_LINK_VALID ? '1' : '0' );
}

static ssize_t z77_set_linkstate(struct device *dev, struct device_attribute *attr,
								 const char *buf, size_t count)
{
	/* is a noop */
	return 0;
}

static DEVICE_ATTR(linkstate, 0644, z77_show_linkstate, z77_set_linkstate );
#endif


/******************************************************************************
 ** setup BD ring in non DMA mode for P511
 *
 * \param dev		\IN net_device struct for this NIC
 *
 * \return			1 if link us up or 0
 */
static int z77_bd_setup_nondma(struct net_device *dev)
{
  unsigned long i=0;
	struct z77_private  *np = netdev_priv(dev);

	/* careful, wipe out at Z077_BDBASE now , not Z077_BASE! */
	memset((unsigned char*)(Z077_BDBASE + Z077_BD_OFFS), 0x00, 0x400 );

	/* Setup Tx BDs */
	for (i = 0; i < Z077_TBD_NUM; i++ ) {

		/* BdAddr = CPU centric BD address view (ioremapped BAR1) */
	np->txBd[i].BdAddr = (unsigned long)((np->ddr_virt+(Z77_ETHBUF_SIZE*i)) + (np->instance * PCIDDR_OFFS ));

		/* hdlDma = IP core centric address view (BAR1 start = 0x00000000) */
	np->txBd[i].hdlDma = (unsigned long)(((Z77_ETHBUF_SIZE * i) | (1<<31))+( np->instance*PCIDDR_OFFS )  );

	Z077_SET_TBD_ADDR(i, np->txBd[i].hdlDma );
		
	/* cleanout the memory */
	memset((unsigned long *)(np->txBd[i].BdAddr), 0x00, Z77_ETHBUF_SIZE);
	/* let an IRQ be generated whenever packet is ready */
	Z077_SET_TBD_FLAG( i, Z077_TBD_IRQ );
	}

	/* Setup Receive BDs */
	for ( i = 0; i < Z077_RBD_NUM; i++ ) {

		/* BdAddr = CPU centric BD address view (ioremapped BAR1) */
    np->rxBd[i].BdAddr = (unsigned long)((np->ddr_virt + (Z77_ETHBUF_SIZE * (Z077_TBD_NUM+i))) + (np->instance * PCIDDR_OFFS ) );

		/* hdlDma = IC cores centric address view (BAR1 start = 0x00000000) */
    np->rxBd[i].hdlDma = (((Z77_ETHBUF_SIZE * (Z077_TBD_NUM+i)) | 1<<31) +
				  (np->instance * PCIDDR_OFFS));

		/* cleanout this memory */
		memset( (char*)(np->rxBd[i].BdAddr), 0, Z77_ETHBUF_SIZE);

		/* ETH core wants physical Addresses */
		Z077_SET_RBD_ADDR( i, np->rxBd[i].hdlDma );
		Z077_SET_RBD_FLAG( i, Z077_RBD_IRQ | Z077_RBD_EMP );

	}

	/* close the Rx/Tx Rings with Wrap bit in each last BD */
	Z077_SET_TBD_FLAG( Z077_TBD_NUM - 1 , Z077_TBD_WRAP );
	Z077_SET_RBD_FLAG( Z077_RBD_NUM - 1 , Z077_RBD_WRAP );

	return(0);
}




/****************************************************************************/
/** z77_bd_setup - perform initialization of buffer descriptors
 *
 * \param dev		\IN net_device struct for this NIC
 *
 * \brief this initializes the buffer descriptors such that PCI DMA - ready
 *        memory chunks are allocated
 *
 */
static int z77_bd_setup(struct net_device *dev)
{

	u32 i=0;
	struct z77_private *np = netdev_priv(dev);
#ifdef CONFIG_PCI
	dma_addr_t 	memPhysDma;
	struct pci_dev		*pcd = np->pdev;
#endif
	dma_addr_t dma_handle = 0;
	void *     memVirtDma = NULL;

	/* careful, wipe out at Z077_BDBASE now , not Z077_BASE! */
	memset((char*)(Z077_BDBASE + Z077_BD_OFFS), 0x00, 0x400);

	/* Setup Tx BDs */
	for ( i = 0; i < Z077_TBD_NUM; i++ ) {
#ifdef CONFIG_PCI
		memVirtDma = pci_alloc_consistent(pcd, Z77_ETHBUF_SIZE, &memPhysDma);
#else
		memVirtDma = kmalloc(Z77_ETHBUF_SIZE, GFP_KERNEL);
		dma_handle = (dma_addr_t)memVirtDma;
#endif
    np->txBd[i].BdAddr = (unsigned long)memVirtDma;

		/* cleanout the memory */
		memset((char*)(memVirtDma), 0, Z77_ETHBUF_SIZE);
		/* let an IRQ be generated whenever packet is ready */
		Z077_SET_TBD_FLAG( i, Z077_TBD_IRQ );
	}

	/* Setup Receive BDs */
	for (i = 0; i < Z077_RBD_NUM; i++ ) {

#ifdef CONFIG_PCI
		memVirtDma = pci_alloc_consistent( pcd, Z77_ETHBUF_SIZE, &memPhysDma);
		dma_handle = pci_map_single( pcd, memVirtDma, (size_t)Z77_ETHBUF_SIZE, PCI_DMA_FROMDEVICE);
#else
		memVirtDma = kmalloc(Z77_ETHBUF_SIZE, GFP_KERNEL);
        dma_handle = (dma_addr_t)memVirtDma;
#endif

		/* leave some headroom for skb->head  */
    np->rxBd[i].BdAddr = (unsigned long)memVirtDma;
		np->rxBd[i].hdlDma = dma_handle;

		/* cleanout the memory */
		memset((char*)(memVirtDma), 0, Z77_ETHBUF_SIZE);

		/* ETH core wants physical Addresses for PCI MemRd/MemWr */
		Z077_SET_RBD_ADDR( i, dma_handle );
		Z077_SET_RBD_FLAG( i, Z077_RBD_IRQ | Z077_RBD_EMP );
	}

	/* close the Rx/Tx Rings with Wrap bit in each last BD */
	Z077_SET_TBD_FLAG( Z077_TBD_NUM - 1 , Z077_TBD_WRAP );
	Z077_SET_RBD_FLAG( Z077_RBD_NUM - 1 , Z077_RBD_WRAP );

	return(0);
}




/****************************************************************************/
/** z77_setup - perform initialization of buffer descriptors
 *
 * \param dev		\IN net_device struct for this NIC
 *
 * \brief this acts as wrapper that calls the adequate setup function
 *        depending on normal DMA use (ethernet frames sent to CPU memory
 *        by DMA from IP core) or non DMA use (currently only P511)
 *
 */
static int z77_setup(struct net_device *dev)
{
	if (nodma)
		return z77_bd_setup_nondma(dev);
	else
		return z77_bd_setup(dev);
}

#if defined(Z77_USE_VLAN_TAGGING)

#if LINUX_VERSION_CODE < VERSION_CODE(3,0,0) 
/****************************************************************************/
/** z77_vlan_rx_register - register a VLAN group ID
 *
 * \param dev		\IN net_device struct for this NIC
 * \param vid		\IN VLAN group ID
 *
 */
static void z77_vlan_rx_register(struct net_device *dev, struct vlan_group *grp)
{
	struct z77_private *np = netdev_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&np->lock, flags);

	Z77DBG(ETHT_MESSAGE_LVL1, "--> %s\n", __FUNCTION__);
	np->vlgrp = grp;

	spin_unlock_irqrestore (&np->lock, flags);

	return;
}
#endif

/****************************************************************************/
/** z77_vlan_rx_add_vid - add a VLAN group ID to this net device
 *
 * \param dev		\IN net_device struct for this NIC
 * \param vid		\IN VLAN group ID
 *
 */
static int z77_vlan_rx_add_vid(struct net_device *dev, unsigned short proto, unsigned short vid)
{

	struct z77_private *np = netdev_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&np->lock, flags);

	Z77DBG(ETHT_MESSAGE_LVL1, "--> %s\n", __FUNCTION__);

	if (!np->vlgrp) {
	  Z77DBG(ETHT_MESSAGE_LVL1, "%s: vlgrp = NULL!\n", __FUNCTION__);
	} else {
    Z77DBG(ETHT_MESSAGE_LVL1, "%s: adding VLAN:%d\n", __FUNCTION__, vid );
	}
	spin_unlock_irqrestore (&np->lock, flags);
  return 0;
}

# if LINUX_VERSION_CODE < VERSION_CODE(3,0,0) 
/****************************************************************************/
/** z77_vlan_rx_kill_vid - delete a VLAN group ID
 *
 * \param dev		\IN net_device struct for this NIC
 * \param vid		\IN VLAN group ID
 *
 */
static void z77_vlan_rx_kill_vid(struct net_device *dev, unsigned short vid)
{
/* -		dev = vlan_group_get_device(grp, vid); */
/* +		dev = __vlan_find_dev_deep(netdev, vid); */
	struct z77_private *np = netdev_priv(dev);
	unsigned long flags;


	spin_lock_irqsave(&np->lock, flags);

	if (!np->vlgrp) {
	  Z77DBG(ETHT_MESSAGE_LVL1, "%s: vlgrp = NULL!\n", __FUNCTION__);
	} else {
	  Z77DBG(ETHT_MESSAGE_LVL1, "%s: killing VLAN:%d (vlgrp entry:%p)\n",
		 __FUNCTION__, vid, vlan_group_get_device(np->vlgrp, vid));

	  vlan_group_set_device(np->vlgrp, vid, NULL);

	}
	spin_unlock_irqrestore (&np->lock, flags);
	return;
}
# endif
#endif


/****************************************************************************/
/** net_rx_err - irq context handler to report Errors
 *
 * \param dev		\IN net_device struct for this NIC
 *
 */
static void z77_rx_err( struct net_device *dev )
{

	int i;
	struct z77_private *np = netdev_priv(dev);

	/* just skip Rx BD Ring backwards so we miss none */
	for (i = Z077_RBD_NUM - 1; i >= 0; i--) {
		if (Z077_GET_RBD_FLAG( i , 0x1ff)) {
			if (Z077_GET_RBD_FLAG( i ,OETH_RX_BD_OVERRUN)) {
				Z77DBG(ETHT_MESSAGE_LVL1, "*** RX: overrun[%d]\n", i);
				np->stats.rx_over_errors++;
			}

			if (Z077_GET_RBD_FLAG( i ,OETH_RX_BD_INVSYMB)) {
				Z77DBG(ETHT_MESSAGE_LVL1, "*** RX: inv symbol[%d]\n", i);
			}

			if (Z077_GET_RBD_FLAG( i ,OETH_RX_BD_DRIBBLE)) {
				Z77DBG(ETHT_MESSAGE_LVL1, "*** RX: dribble[%d]\n", i);
				np->stats.rx_frame_errors++;
			}

			if (Z077_GET_RBD_FLAG( i ,OETH_RX_BD_TOOLONG)) {
				Z77DBG(ETHT_MESSAGE_LVL1, "*** RX: too long[%d]\n", i);
				np->stats.rx_length_errors++;
			}

			if (Z077_GET_RBD_FLAG( i ,OETH_RX_BD_SHORT)) {
				Z77DBG(ETHT_MESSAGE_LVL1, "*** RX: too short[%d]\n", i);
				np->stats.rx_length_errors++;
			}

			if (Z077_GET_RBD_FLAG( i ,OETH_RX_BD_CRCERR)) {
				Z77DBG(ETHT_MESSAGE_LVL1, "*** RX: CRC err[%d]\n", i);
				np->stats.rx_crc_errors++;
			}

			if (Z077_GET_RBD_FLAG( i ,OETH_RX_BD_LATECOL)) {
				Z77DBG(ETHT_MESSAGE_LVL1, "*** RX: late coll[%d]\n", i);
			}

			np->stats.rx_errors++;

		    /* Flags are reported, clear them */
			Z077_CLR_RBD_FLAG( i , OETH_RX_BD_OVERRUN | OETH_RX_BD_INVSYMB | \
							   OETH_RX_BD_DRIBBLE | OETH_RX_BD_TOOLONG | \
							   OETH_RX_BD_SHORT | OETH_RX_BD_CRCERR | \
							   OETH_RX_BD_LATECOL );
		}
	}
}


/****************************************************************************/
/** net_tx_err - irq context handler to report Errors
 *
 * \param dev		\IN net_device struct for this NIC
 *
 */
static void z77_tx_err( struct net_device *dev)
{
	int i;
	struct z77_private *np = netdev_priv(dev);

	/* simply skip Tx BD Ring backwards */
	for (i = Z077_TBD_NUM-1; i >= 0; i--) {
		if (Z077_GET_TBD_FLAG( i , OETH_TX_BD_DEFER | OETH_TX_BD_CARRIER
							   | OETH_TX_BD_UNDERRUN | OETH_TX_BD_RETLIM
							   | OETH_TX_BD_LATECOL)) {

			if (Z077_GET_TBD_FLAG( i, OETH_TX_BD_DEFER)) {
				Z77DBG(ETHT_MESSAGE_LVL1, "*** TX: defered frame[%d]\n", i);
			}

			if (Z077_GET_TBD_FLAG( i, OETH_TX_BD_CARRIER)) {
				Z77DBG(ETHT_MESSAGE_LVL1, "*** TX: Carrier lost[%d]\n", i);
				np->stats.tx_carrier_errors++;
			}

			if (Z077_GET_TBD_FLAG( i, OETH_TX_BD_UNDERRUN)) {
				Z77DBG(ETHT_MESSAGE_LVL1, "*** TX: underrun[%d]\n", i);
				np->stats.tx_fifo_errors++;
			}

			if (Z077_GET_TBD_FLAG( i, OETH_TX_BD_RETLIM)) {
				Z77DBG(ETHT_MESSAGE_LVL1, "*** TX: retrans limit[%d]\n", i);
				np->stats.tx_aborted_errors++;
			}

			if (Z077_GET_TBD_FLAG( i, OETH_TX_BD_LATECOL)) {
				Z77DBG(ETHT_MESSAGE_LVL1, "*** TX: late coll[%d]\n", i);
				np->stats.tx_window_errors++;
			}

			np->stats.tx_errors++;
			np->stats.collisions += Z077_GET_TBD_FLAG(i, OETH_TX_BD_RETRY) >> 4;

			Z077_CLR_TBD_FLAG( i, OETH_TX_BD_DEFER | OETH_TX_BD_CARRIER | \
							   OETH_TX_BD_UNDERRUN | OETH_TX_BD_RETLIM | \
							   OETH_TX_BD_LATECOL );
		}
	}
}


/****************************************************************************/
/** z77_reset - reset device with asynchronous global Reset Register 0x54
 *
 * \param dev		\IN net_device struct for this NIC
 *
 */
static void z77_reset( struct net_device *dev )
{

	u32 tout = IP_CORE_TIMEOUT;
	u32 idle = (OETH_MODER_RX_IDLE | OETH_MODER_TX_IDLE);

    /* stop receiving/transmitting */
    Z077_CLR_MODE_FLAG(OETH_MODER_RXEN | OETH_MODER_TXEN );

	/* loop until Rx/Tx idle (MODER[17:18]) = 0b11 */
	while (((Z77READ_D32(Z077_BASE,Z077_REG_MODER) & idle) != idle) && tout) {
		/* dont block PCI bus completely, poll IDLE every 50 us */
		udelay(50);
		tout --;
	}

	if (!tout) {
		printk (KERN_ERR "*** z77_reset: ETH core not idle after reset!\n");
	}

}

#if defined(MEN_Z77_USE_OWN_PHYACCESS)
/****************************************************************************/
/** z77_phy_init - initialize and configure the PHY devices
 *
 * \param dev		\IN net_device struct for this NIC
 *
 * \brief
 * This routine scans, initializes and configures PHY devices.
 */
static int z77_phy_init(struct net_device *dev)
{
    u16	dat = 0;
	u8	phyAddr;		/* address of a PHY */
    u32	found = FALSE;	/* no PHY has been found */

	struct z77_private *np = netdev_priv(dev);
	phyAddr = np->mii_if.phy_id;

	/* reset the PHY */
	z77_phy_reset( dev, phyAddr	);

	/* check which PHY is there */
	if (z77_phy_identify(dev, phyAddr)) {
		return (-ENODEV);
	}

	/* on F218 revert the Link and activity LED */
	dat = z77_mdio_read(dev, phyAddr, MII_PHYSID2 );
	if ((dat == PHY_ID2_KSZ8041_1) || (dat == PHY_ID2_KSZ8041_2) ) {
		printk( KERN_INFO "found PHY KSZ8041. reverting Link/Act LED" );
		dat = z77_mdio_read(dev, np->mii_if.phy_id , 0x1e );
		dat |=0x4000; /* just set bit14, bit 15 is reserved (datasheet p.29) */
		z77_mdio_write( dev, phyAddr, 0x1e, dat );
	}

	found = TRUE;

	/* disable powerdown mode */
	dat = z77_mdio_read(dev, phyAddr, MII_100_BASE_TX_PHY );
	dat &=~(MII_100BASE_PWR);
	z77_mdio_write(dev, phyAddr, MII_100_BASE_TX_PHY, dat);

	/* set desired Phy mode: auto-negotiation or fixed. */
	if (z77_init_phymode (dev, phyAddr) == 0)
		return (0);

    if (!found)
		return (-ENODEV);

    /* if we're here, none of the PHYs could be initialized */
	printk( KERN_ERR "*** z77_phy_init check cable connection \n");
    return (-ENODEV);
}



/****************************************************************************/
/** z77_phy_identify - probe the PHY device
 *
 * \param dev		\IN net_device struct for this NIC
 * \param phyAddr	\IN Address of used PHY, currently 1
 *
 * \return			zero if Phy found, otherwise nonzero
 * \brief
 * This routine probes the PHY device by reading its PHY Identifier Register
 */
static int z77_phy_identify(struct net_device *dev,u8 phyAddr )
{
	u32 i=0;
    u16	data;		/* data to be written to the control reg */
	u16	id2;
    data = z77_mdio_read(dev, phyAddr, MII_PHYSID1 );

	while (z077PhyAttachTbl[i].ident !=0xffff) {
		if (data == z077PhyAttachTbl[i].ident ) {
			id2 = z77_mdio_read(dev, phyAddr, MII_PHYSID2 );
			printk( KERN_INFO "PHY %s found. (MII_PHYSID1: 0x%04x MII_PHYSID2: 0x%04x)\n",
					z077PhyAttachTbl[i].name, data, id2 );
			return (0);
		}
		i++;
	}
	/* found no known PHY in table, error */
	printk( KERN_ERR "*** z77_phy_identify: unknown Phy ID 0x%04x!\n", data );
	return (-ENODEV);
}


/****************************************************************************/
/** z77_phy_reset
 *
 * \param dev		\IN net_device struct for this NIC
 * \param phyAddr	\IN Address of used PHY, currently 1
 *
 * \brief
 * This routine send a Reset command to the PHY specified in
 * the parameter phyaddr.
 *
 * \return 0 always
 */
static int z77_phy_reset( struct net_device *dev, u8 phyAddr )
{
	u16 dat = 0;
	u32 tmp = 0;

	dat = z77_mdio_read(dev, phyAddr, MII_BMCR );
	tmp = Z77READ_D32( Z077_BASE , Z077_REG_MIISTATUS );
	z77_mdio_write( dev, phyAddr, MII_BMCR, BMCR_RESET );
	dat = z77_mdio_read( dev, phyAddr, MII_BMCR );

	return(0);
}



/****************************************************************************/
/** z77_init_phymode - Set Phy Mode and Flags according to given mode
 *
 * \param dev		\IN net_device struct for this NIC
 * \param phyAddr	\IN PHY Chip address, one of 0 to 31. Is 1 at EM01
 *
 * \return 0 or error code
 */
static int z77_init_phymode (struct net_device *dev, u8 phyAddr)
{

	unsigned bDoAutoneg = 0;
	unsigned int dat=0;
	struct ethtool_cmd cmd;
	int res = 0;
	char holder[PHY_MODE_NAME_LEN+1];
	struct z77_private *np = netdev_priv(dev);

	Z77DBG(ETHT_MESSAGE_LVL1, "--> %s(phyAddr=%d)\n", __FUNCTION__, phyAddr);

	/* some default settings */
	cmd.port 		 	= PORT_MII;
	cmd.transceiver  	= XCVR_INTERNAL;
	cmd.phy_address  	= phyAddr;
	cmd.autoneg      	= AUTONEG_DISABLE;

	strncpy(holder, G_phyMode[np->instCount], PHY_MODE_NAME_LEN);

	if ( np->modCode == CHAMELEON_16Z077_ETH ) {
		printk( KERN_INFO "Z077 fix: forcing 10HD due to bandwith limits\n");
		strncpy(holder, "10HD", 7);
	}
	/* be paranoid.. */
	holder[PHY_MODE_NAME_LEN]=0;
	printk( KERN_INFO "MEN ETH instance %s: using phy mode '%s'\n", dev->name, holder);

	if (!(strncmp(holder, "10HD", 	4))) {
		np->mii_if.full_duplex=0;
		np->mii_if.force_media=1;
		cmd.speed 	= SPEED_10;
		cmd.duplex 	= DUPLEX_HALF;
	}
	else if (!(strncmp(holder,"10FD", 4))) {
		np->mii_if.full_duplex=1;
		np->mii_if.force_media=1;
		cmd.speed 	= SPEED_10;
		cmd.duplex 	= DUPLEX_FULL;
	}
	else if (!(strncmp(holder,"100HD", 5))) {
		np->mii_if.full_duplex=0;
		np->mii_if.force_media=1;
		cmd.speed 		= SPEED_100;
		cmd.duplex 	= DUPLEX_HALF;
	}
	else if (!(strncmp(holder,"100FD", 5))) {
		np->mii_if.full_duplex=1;
		np->mii_if.force_media=1;
		cmd.speed 	= SPEED_100;
		cmd.duplex 	= DUPLEX_FULL;
	}
	else if (!(strncmp(holder,"AUTO", 4))) {
		np->mii_if.full_duplex=1;
		np->mii_if.force_media=0;
		cmd.speed 		= SPEED_100;
		cmd.duplex 		= DUPLEX_FULL;
		cmd.autoneg 	= AUTONEG_ENABLE;
		bDoAutoneg 		= 1;
	} else {
		printk(KERN_ERR "*** invalid mode parameter '%s'\n", holder);
		return -EINVAL;
	}
	printk(KERN_INFO "PHY set to %s\n", holder);

	/* apply desired mode or autonegotiate */
	if ( bDoAutoneg ) {

		/* set NWAYEN bit 0.12 accordingly */
		dat = z77_mdio_read(dev, np->mii_if.phy_id, MII_BMCR );
		dat |=(1<<12); /* bit 0.12 = autonegotiation enable */
		z77_mdio_write( dev, np->mii_if.phy_id, MII_BMCR, dat );
		if ( (res = mii_nway_restart(&np->mii_if )) ) {
			printk(KERN_ERR "*** setting autoneg. PHY mode failed\n");
			return -EINVAL;
		}
	} else {
		/* set MODER[10] (HD/FD) according to passed duplex setting */
		if (np->mii_if.full_duplex)
			Z077_SET_MODE_FLAG(OETH_MODER_FULLD);
		else
			Z077_CLR_MODE_FLAG(OETH_MODER_FULLD);

		if ((res = mii_ethtool_sset(&np->mii_if, &cmd))) {
			printk(KERN_INFO "PHY setting fixed mode failed - fixed MEN Phy\n" );
		}
	}

	Z77DBG(ETHT_MESSAGE_LVL1, "<-- %s()\n", __FUNCTION__);
	return 0;
}

/****************************************************************************/
/** Do the complete Autonegotiation for the used Ethernet PHY
 *
 * \param dev			\IN net_device struct for this NIC
 *
 * \return 0 or error code
 */
static int z77_do_autonegotiation(struct net_device *dev)
{
	int rv = 0;

	if (z77_phy_init(dev)) {
		printk("*** PHY Initialization failed!\n");
		rv = -ENODEV;
	}

	return rv;
}
#endif


/****************************************************************************/
/** remove the NIC from Kernel
 *
 * \param dev			\IN net_device struct for this NIC
 *
 * \return 0 or error code
 */
static void cleanup_card(struct net_device *dev)
{
	free_irq(dev->irq, dev);
	release_region(dev->base_addr, Z77_CFGREG_SIZE);
}


/****************************************************************************/
/** Check for a network adapter of this type, and return '0' if one exists.
 *
 * \param dev			\IN net_device struct for this NIC
 *
 * \return 0 or error code
 */
static int __init probe_z77(struct net_device *dev)
{
	struct z77_private *np;
	np 	= netdev_priv(dev);
	spin_lock_init(&np->lock);

#if LINUX_VERSION_CODE <= VERSION_CODE(2,6,23)
	SET_MODULE_OWNER(dev);
#endif

	printk(KERN_INFO "%s: %s found at 0x%08lx\n",
		   dev->name, cardname, dev->base_addr);

	/* no longer existing in 2.6.30 and up: */
#if LINUX_VERSION_CODE  < VERSION_CODE(2,6,30)
	dev->open				= z77_open;
	dev->stop				= z77_close;
	dev->hard_start_xmit	= z77_send_packet;
	dev->get_stats			= z77_get_stats;
	dev->tx_timeout			= z77_tx_timeout;
	dev->do_ioctl			= z77_ioctl;
	dev->get_stats			= z77_get_stats;
	dev->set_multicast_list	= z77_set_rx_mode;
#else
	dev->netdev_ops 		= &z77_netdev_ops;
#endif
    dev->watchdog_timeo		= MY_TX_TIMEOUT;

#if LINUX_VERSION_CODE <= VERSION_CODE(2,6,23)
	dev->poll				= z77_poll;
	dev->weight 			= Z077_WEIGHT;
#endif

	/* use PHY address from passed module parameter */
	/* np->mii_if.phy_id 		= phyadr[t]; */
	np->mii_if.phy_id_mask 	= 0x1f;
	np->mii_if.reg_num_mask = 0x1f;
	np->mii_if.dev 			= dev;
	np->mii_if.mdio_read 	= z77_mdio_read;
	np->mii_if.mdio_write 	= z77_mdio_write;

	/* YES, we support the standard ethtool utility */
#if LINUX_VERSION_CODE < VERSION_CODE(3,16,0)
	SET_ETHTOOL_OPS(dev,    &z77_ethtool_ops);                                                                                  
#else
	dev->ethtool_ops = &z77_ethtool_ops;
#endif
	/* Data setup done, now setup Connection */
	if (chipset_init(dev, 0)) {
		printk(KERN_ERR "*** probe_z77: Ethernet core init failed!\n");
		return(-ENODEV);
	} else
		return(0);
}

/****************************************************************************/
/** Timeout handler when no scheduled ETH irq arrived
 *
 * \param work		\IN handle of work_struct 
 *
 * \return -
 *
 */
static void z77_reset_task(struct work_struct *work)
{
	struct z77_private *np = container_of(work, struct z77_private, reset_task);
    struct net_device *dev = np->dev;
	struct ethtool_cmd ecmd = {0};
	int settings_saved=0;

	
	Z077_DISABLE_IRQ( Z077_IRQ_ALL );

	netif_tx_disable(dev);

	printk(KERN_WARNING "%s: NETDEV WATCHDOG timeout! (%s)\n",
		   dev->name, __FUNCTION__ );

	if (np->msg_enable) {
		printk(KERN_WARNING "Current register settings before restart:\n");
		z77_regdump(dev);
	}

	/* save settings */
	settings_saved = !z77_ethtool_get_settings(dev, &ecmd);

	z77_close(dev);
	/* Try to restart the adaptor. */
	z77_open(dev);

	/* restore settings */
	if (settings_saved) {
		z77_ethtool_set_settings(dev, &ecmd);
	}

	np->stats.tx_errors++;

	/* If we have space available to accept new transmits, do so */
	if (!tx_full(dev))
		netif_wake_queue(dev);

#ifdef DO_NETDEV_TIMEOUT_TEST
	/* we get here when a TX timeout occured */
	netif_wake_queue(dev);
	Z077_ENABLE_IRQ( OETH_INT_TXB );
#endif

}


/****************************************************************************/
/** Timeout handler when no scheduled ETH irq arrived
 *
 * \param dev			\IN net_device struct for this NIC
 *
 * \return -
 *
 */
static void z77_tx_timeout(struct net_device *dev)
{
	struct z77_private *np = netdev_priv(dev);
	Z77DBG( ETHT_MESSAGE_LVL1, "z77_tx_timeout called!\n");
	/* place reset outside of interrupt context (timer = soft irq!)*/
	schedule_work(&np->reset_task);
}



/****************************************************************************/
/** z77_process_rx - process each nonempty Rx BD
 *
 * \param dev	 	\IN net_device struct of this interface
 * \param weight	\IN allowed # of packets to process in a call
 *
 * \brief
 * Without Rx position info determining oldest Rx frame
 * is complicated, several basic situations are possible:
 *
 * \verbatim
 * <-- BD fill direction of Z87 core -----
 * 63      RX1      32 31       RX0      0
 *  --------------------------------------
 * |                  |             oXXXXX|
 *  -------------------------------------^
 * |              oXXX|XXXXXXXXXXXXXX     |
 *  --------------------------------^-----
 * |       oXXXXXXX   |                   |
 *  --------------^-----------------------
 * |XXXXXXX           |                  o|
 *  ------^-------------------------------
 * |XX                |             oXXXXX|
 *  -^------------------------------------
 * |XXXXXXXXXXXXXXXXXX|XX              oXX|
 *  --------------------^-----------------
 * |XX            oXXX|XXXXXXXXXXXXXXXXXXX|
 *  -^------------------------------------
 *
 * ^ = oldest Rx frame in this cycle, which is to be passed to stack first
 * o = position available from new Rx count register 0x70
 *
 * Assumption made: always only one connected region of full BDs ('x') exists
 *
 * simplified Algorithm: 1. skip from Rx BD63 backwards until first full-empty
 *                          transition found. This is oldest Rx BD (startpos)
 *                       2. from startpos skip up and pass every nonempty BD
 *                          to network stack
 * \endverbatim
 *
 * PS: should ever Rx BD organisation change, this needs to be reworked.
 *     Code in here implies 64 Rx BDs in 2x32bit Registers!.
 */
static int z77_process_rx( struct net_device *dev, int weight )
{
	int	nrframes = 0;
	unsigned int start_pos=0, rx0, rx1, emp_n=0, emp_n1=0;
	int i;

	/* For this poll-cycle we check RX BDs only here! */
	rx0 = ~Z77READ_D32( Z077_BASE, Z077_REG_RXEMPTY0 );
	rx1 = ~Z77READ_D32( Z077_BASE, Z077_REG_RXEMPTY1 );

	if(!rx0 && !rx1){
		/* all empty, should never happen.. if yes just return */
		/* WARN_ONCE(1, "%s: RX BDs are all empty!\n", __func__); */
		return 0;
	}

	/* 1.) find oldest nonempty Rx BD in BDs */
	if(!~rx0 && !~rx1){
		/* all full, algorithm will not work! */
		/* -> we try to get position from FPGA */
		u32 rx_bd_stat = Z77READ_D32( dev->base_addr, Z077_REG_RXBDSTAT );
		start_pos = rx_bd_stat & 63; /* start_pos=0 if reg is not implemented */
	}
	else {
		/* start algorithm... */
		for (i = 63; i >=0; i--) {
			/* the 64 Rx BDs are split in 2 x 32bit registers, check boundaries */
			if (i > 32) { /* 63..33  RX1 only */
				emp_n  = (rx1 & ( 1 << (i-32)   ));
				emp_n1 = (rx1 & ( 1 << (i-32-1) ));
			}
			if (i == 32) { /* 32:  check border RX1/RX0 */
				emp_n  = ( rx1 & 0x00000001 );
				emp_n1 = ( rx0 & 0x80000000 );
			}
			if ( (i < 32) && ( i > 0) ) { /* 31..1 : RX0 only */
				emp_n  = (rx0 & ( 1 <<    i ));
				emp_n1 = (rx0 & ( 1 << (i-1)));
			}
			if ( i == 0) { /* 0->63 */
				emp_n  = (rx0 & 0x00000001 );
				emp_n1 = (rx1 & 0x80000000 );
			}

			/* if at this position an full-empty border occurs, its start pos. */
			if ( (emp_n != 0) && (emp_n1 == 0) ) {
				/* rx0,rx1 is inversed RX[01]_EMPTY */
				start_pos = i;
				break;
			}
		}
	}

	/* 2.) Now skip from start_pos forward until empty RX BDs occur again */
	while (  nrframes < weight ) {
		/* pass oldest nonempty packet */
		z77_pass_packet( dev, start_pos );
		nrframes++;

		start_pos++;
		start_pos %= Z077_RBD_NUM;

		/* are we done ? */
		if ( (start_pos < 32) &&  ( (rx0 & (1 <<  start_pos )    ) == 0 ) )
			break;
		if ( (start_pos >= 32) && ( (rx1 & (1 << (start_pos-32) )) == 0 ) )
			break;
	}
	return nrframes;
}


#if LINUX_VERSION_CODE > VERSION_CODE(2,6,23)
/****************************************************************************/
/** z77_poll - Rx poll function to support the NAPI functionality
 *
 * \param napi			\IN NAPI struct for this NIC
 * \param budget		\IN allowed # of packets to process in a call
 *
 * \return 0 if all packets were processed or 1 of not all was processed
 *
 * \brief  The poll routine works according to
 *         linux/Documentation/networking/NAPI_HOWTO.txt
 *         addendum ts: NAPI_HOWTO.txt was removed from vanilla kernel in 2.6.24
 */
/* ts: weight == budget! see net/core/dev.c */
static int z77_poll(struct napi_struct *napi, int budget)
{
	int npackets = 0;
	struct z77_private *np = container_of(napi, struct z77_private, napi);
    struct net_device *dev = np->dev;

	Z77DBG( ETHT_MESSAGE_LVL3, "--> z77_poll:\n");
	npackets = z77_process_rx( dev, budget );

	if ( npackets < budget ) { /* we are done, for NOW */

#if LINUX_VERSION_CODE > VERSION_CODE(2,6,28) 
		napi_complete(napi);
#else
		netif_rx_complete(dev, napi);
#endif
		/* acknowledge last Rx IRQ and expect new interrupts */
		Z77WRITE_D32(Z077_BASE, Z077_REG_INT_SRC, OETH_INT_RXF );
		if (np->gotBusy) {
			Z77WRITE_D32(Z077_BASE, Z077_REG_INT_SRC, OETH_INT_BUSY );
			Z077_DISABLE_IRQ( OETH_INT_BUSY );
			np->gotBusy=0;
		}
		Z077_ENABLE_IRQ(OETH_INT_RXF);
	}
	return npackets;
}

#else

static int z77_poll(struct net_device *dev, int *budget)
{
	int npackets = 0, quota = min(dev->quota, *budget);
	unsigned int rx0, rx1;
	struct z77_private *np = netdev_priv(dev);

	Z77DBG( ETHT_MESSAGE_LVL3, "--> z77_poll:\n");
	/* check the nonempty RX BDs to be passed to Network Stack */
	rx0 = ~Z77READ_D32( Z077_BASE, Z077_REG_RXEMPTY0 );
	rx1 = ~Z77READ_D32( Z077_BASE, Z077_REG_RXEMPTY1 );

	npackets = z77_process_rx( dev, quota );

	/*If we processed all packets,were done; tell kernel and reenable ints */
	*budget-=npackets;
	dev->quota -=npackets;

	/* work left or new packet arrived meanwhile ? */
	rx0 = ~Z77READ_D32( Z077_BASE, Z077_REG_RXEMPTY0 );
	rx1 = ~Z77READ_D32( Z077_BASE, Z077_REG_RXEMPTY1 );
	if ( !rx0 && !rx1 ) { /* we are done, for NOW */
		netif_rx_complete(dev);
		Z77DBG( ETHT_MESSAGE_LVL3,"<-- z77_poll:\n");

		/* acknowledge last Rx IRQ and expect new interrupts */
		Z77WRITE_D32(Z077_BASE, Z077_REG_INT_SRC, OETH_INT_RXF );
		if (np->gotBusy) {
			Z77WRITE_D32(Z077_BASE, Z077_REG_INT_SRC, OETH_INT_BUSY );
			Z077_DISABLE_IRQ( OETH_INT_BUSY );
			np->gotBusy=0;
		}
		Z077_ENABLE_IRQ(OETH_INT_RXF);
		return 0;
	}
	return 1;
}
#endif


/****************************************************************************/
/** z77_open - open the ethernet device for first usage
 *
 * \param dev			\IN net_device struct for this NIC
 *
 * \return 0 or error code
 *
 * \brief
 * Open/initialize the Ethernet device.
 * This routine should set everything up anew at each open, even
 * registers that should only need to be set once at boot, so that
 * there is non-reboot way to recover if something goes wrong.
 */
static int z77_open(struct net_device *dev)
{

	struct z77_private *np = netdev_priv(dev);

	Z77DBG( ETHT_MESSAGE_LVL1, "-> %s(dev->name='%s') \n",
			__FUNCTION__, 	dev->name );

	/* do PHY/MAC initialization with forced mode or autonegotiation */
	if (chipset_init(dev, 1)) {
		printk(KERN_ERR "*** z77_open: initializing Ethernet core failed!\n"); ;
		return(-ENODEV);
	}
	/* setup the Tx/Rx buffer descriptors */
	z77_setup(dev);

	/* clear any pending spurious IRQs */
	Z77WRITE_D32( Z077_BASE, Z077_REG_INT_SRC, 0x7f );

	/* hook in the Interrupt handler */
#if LINUX_VERSION_CODE > VERSION_CODE(2,6,18)
	if (request_irq(dev->irq, z77_irq, IRQF_SHARED, cardname, dev)) {
#else
	if (request_irq(dev->irq, z77_irq, SA_SHIRQ, cardname, dev)) {
#endif
		printk(KERN_ERR "*** %s: unable to get IRQ %d.\n", dev->name, dev->irq);
		return -ENOMEM;
	}

#if LINUX_VERSION_CODE > VERSION_CODE(2,6,23)
	napi_enable(&np->napi);
#endif

	/*	enable IRQs we currently use: Rx[-ERR], Tx[-ERR], Busy */
	Z077_ENABLE_IRQ( Z077_IRQ_ALL );

	/* enable receiving and transmitting */
	Z077_SET_MODE_FLAG(OETH_MODER_RXEN | OETH_MODER_TXEN );

	np->open_time = jiffies;

#if !defined(MEN_Z77_USE_OWN_PHYACCESS)
	phy_start_aneg(np->phy_dev);
	/* initial link check */
	phy_start(np->phy_dev);
#else
	/* (re-)kick off link change detection */
	np->timer.expires = jiffies + np->timer_offset;
	add_timer(&np->timer);
#endif

	/* and let the games begin... */
	netif_start_queue(dev);
	np->flags |= IFF_UP;

	Z77DBG( ETHT_MESSAGE_LVL1, "<-- %s()\n", __FUNCTION__ );
	return 0;
}



/*******************************************************************/
/** send a ready made ethernet frame
 *
 * \param dev			\IN net_device struct for this NIC
 * \param skb			\IN struct skbuf with data to transmit
 *
 * \return 				0 or error code
 *
 * \brief
 * The OpenCore MAC Spec states about transmitting:
 *
 *   4.2.3 Frame Transmission
 *   To transmit the first frame, the RISC must do several things, namely:
 *   Store the frame to the memory. Associate the Tx BD in the Ethernet MAC
 *   core with the packet written to the memory ( length, pad, crc, etc.).
 *
 *	 Enable the TX part of the Ethernet Core by setting the TXEN bit to 1.
 *	 As soon as the Ethernet IP Core is enabled, it continuously reads
 *	 the first BD. Immediately when the descriptor is marked as ready, the
 *	 core reads the pointer to the memory storing the associated data and
 *	 starts then reading data to the internal FIFO. Atthe moment the FIFO
 *	 is full, transmission begins.
 *   At the end of the transmission, the
 *	 transmit status is written to the buffer descriptor andan interrupt might
 *	 be generated (when enabled). Next, two events might occur (according to
 *	 the WR bit (wrap) in the descriptor):
 *
 *   o If the WR bit has not been set,
 *	   the BD address is incremented, the next descriptor isloaded, and the
 *	   process starts all over again (if next BD is marked as ready).
 *   o If the WR bit has been set, the first BD address (base) is
 *     loaded again. As soon as the BD is marked as ready, transmission
 *     will start.
 */
static int z77_send_packet(struct sk_buff *skb, struct net_device *dev)
{

	struct z77_private *np 	= 	netdev_priv(dev);
	unsigned char 	*buf 	= 	skb->data;
	u32 txbEmpty 			= 	0;
#if defined(Z77_USE_VLAN_TAGGING) 
	unsigned int vlan_id	= 	0;
	unsigned int vlan_tag	=   0;
#endif
	unsigned int frm_len	=   0;
	int i 					= 	0;
	unsigned char 	idxTx 	=   0;
	dma_addr_t dma_handle 	= 	0;
#ifndef NIOS_II
	u8* dst = NULL;
	u8* src = NULL;
#endif

    /* place Tx request in the recent Tx BD */
	idxTx 	= np->nCurrTbd;

#ifdef DO_NETDEV_TIMEOUT_TEST
	np->nCount++;
	/* printk(KERN_CRIT "nCount = %d...\n", np->nCount); */
	if ((np->nCount > 10) && (np->doNDOtest == 0)) {
		printk(KERN_CRIT "*** DO_NETDEV_TIMEOUT_TEST\n");
		np->doNDOtest = 1;
		Z077_DISABLE_IRQ( OETH_INT_TXB );
		netif_stop_queue(dev);
	}	
#endif

	/* some statistics (ok they are old, but better collect them now than
	   leave them totally) */
	np->stats.collisions += Z077_GET_TBD_FLAG( idxTx, OETH_TX_BD_RETRY) >> 4;

	/* Check if this Tx BD we use now is empty. If not -> drop . */
	if ( np->modCode == CHAMELEON_16Z077_ETH ) {
		if (!Z077_GET_TBD_FLAG( idxTx, OETH_TX_BD_READY )) {
			netif_stop_queue(dev);
			np->stats.tx_dropped++;
			/* free this skb */
			dev_kfree_skb(skb);

# if LINUX_VERSION_CODE < VERSION_CODE(2,6,32)
			return NETDEV_TX_BUSY;
#else
			return 1;
#endif
		}
	} else { /* Z87 Core with extra TXBd empty Flags */
		if ( idxTx < 32 )
			txbEmpty = Z77READ_D32(Z077_BASE, Z077_REG_TXEMPTY0) & (1<<idxTx);
		else
			txbEmpty = Z77READ_D32(Z077_BASE, Z077_REG_TXEMPTY1) &
				(1 << (idxTx-32));

		if (!txbEmpty) { /* congestion? */
			netif_stop_queue(dev);
			np->stats.tx_dropped++;
			
			/* free this skb */
			dev_kfree_skb(skb);

#if LINUX_VERSION_CODE < VERSION_CODE(2,6,32)
			return NETDEV_TX_BUSY;
#else
			return 1;
#endif
		}
	}

	Z77DBG(ETHT_MESSAGE_LVL2, "z77_send_packet[%d] len 0x%04x DMAadr %08x\n",
		   idxTx, skb->len, np->txBd[idxTx].hdlDma );

#ifdef NIOS_II
	memcpy((void*)(np->txBd[idxTx].BdAddr | NO_CACHED_MEM), (void*)buf, skb->len);
	Z077_SET_TBD_ADDR( idxTx, np->txBd[idxTx].hdlDma /* phys.addr */ );
#else
    if (!nodma) {
		dma_handle = pci_map_single( np->pdev, (void*)(np->txBd[idxTx].BdAddr),
									 Z77_ETHBUF_SIZE, PCI_DMA_TODEVICE );
		np->txBd[idxTx].hdlDma = dma_handle;
		Z077_SET_TBD_ADDR( idxTx, dma_handle);
	}

	src 	= (u8*)buf;
	dst 	= (u8*)np->txBd[idxTx].BdAddr;
	frm_len = skb->len;

# if defined(Z77_USE_VLAN_TAGGING) 
	/* VLAN or regular frame ? */
	if ( vlan_tx_tag_present(skb)) {

		vlan_id = vlan_tx_tag_get(skb);
		vlan_tag = htonl((ETH_P_8021Q << 16) | vlan_id);

		Z77DBG(ETHT_MESSAGE_LVL2, "VLAN frame: ID 0x%04x\n", vlan_id);

		/* copy 12 byte dest/src MAC addresses  */
		memcpy(dst, src, ETH_SRCDST_MAC_SIZE );

		/* insert 4 byte VLAN info */
		memcpy(dst+ETH_SRCDST_MAC_SIZE, &vlan_tag, VLAN_TAG_SIZE);

		/* insert rest of the frame */
		memcpy(dst + ETH_SRCDST_MAC_SIZE + VLAN_TAG_SIZE,
			   src + ETH_SRCDST_MAC_SIZE, frm_len - ETH_SRCDST_MAC_SIZE);
		frm_len += VLAN_TAG_SIZE;
	} else {
		Z77DBG(ETHT_MESSAGE_LVL2, "standard frame:\n");
		memcpy(dst, src, skb->len);
	}
# else
	Z77DBG(ETHT_MESSAGE_LVL2, "standard frame:\n");
	memcpy(dst, src, skb->len);
# endif

	Z077_SET_TBD_LEN(  idxTx, frm_len );

	/* sync the mem ranges */
	pci_dma_sync_single_for_cpu( np->pdev, dma_handle, Z77_ETHBUF_SIZE, PCI_DMA_TODEVICE);
#endif

	/* very verbose debugging on? then dump frame */
	if ( np->msg_enable >= ETHT_MESSAGE_LVL3 ) {
		dst	= (u8*)np->txBd[idxTx].BdAddr;
		for (i=0; i < frm_len; i++) {
			if (!(i%16))
				Z77DBG(ETHT_MESSAGE_LVL3, "\n0x%03x: ", i);
			Z77DBG(ETHT_MESSAGE_LVL3, "%02x ", (unsigned char)(*dst++));
		}
		Z77DBG(ETHT_MESSAGE_LVL3, "\n");
	}

	/*
	 * finally kick off transmission
	 */

	if ( np->modCode == CHAMELEON_16Z077_ETH ) {
		/* Open Core: Set TX bit within TxBD  */
		Z077_SET_TBD_FLAG( idxTx, OETH_TX_BD_READY );
	} else {
		/* MEN cores maintain separate TX Empty Registers */
		if (idxTx < 32)
			Z77WRITE_D32(Z077_BASE, Z077_REG_TXEMPTY0, 1 << idxTx );
		else
			Z77WRITE_D32(Z077_BASE, Z077_REG_TXEMPTY1, 1 << (idxTx - 32));
	}

	dev->trans_start = jiffies;
	np->stats.tx_bytes += skb->len;

	/* .. and free this skb */
	dev_kfree_skb(skb);

	/* go to next Tx BD */
	np->nCurrTbd++;
	np->nCurrTbd %= Z077_TBD_NUM;

	/* Check if the next Tx BD is empty. If not -> stall . */
	idxTx = np->nCurrTbd;
	if ( np->modCode == CHAMELEON_16Z077_ETH ) {
		if (Z077_GET_TBD_FLAG( idxTx, OETH_TX_BD_READY )) {
			netif_stop_queue(dev);
			Z77DBG(ETHT_MESSAGE_LVL2, "%s: stopping queue\n", __FUNCTION__ );
		}
	} else { /* Z87 Core with extra TXBd empty Flags */
		if ( idxTx < 32 )
			txbEmpty = Z77READ_D32(Z077_BASE, Z077_REG_TXEMPTY0) & (1<<idxTx);
		else
			txbEmpty = Z77READ_D32(Z077_BASE, Z077_REG_TXEMPTY1) &
				(1 << (idxTx-32));

		if (!txbEmpty) { /* congestion? */
			netif_stop_queue(dev);
			Z77DBG(ETHT_MESSAGE_LVL2, "%s: stop_queue\n", __FUNCTION__ );
		}
	}

# if LINUX_VERSION_CODE < VERSION_CODE(2,6,32)
	return NETDEV_TX_OK;
#else
	return 0;
#endif
}

#ifdef MEN_MM1

/*******************************************************************/
/** perform a SMBus cycle on EEprom
 *
 * \param dev	\IN net_device struct for this NIC
 * \param c	    \IN command char to describe which cycle to perform
 *
 * \return 0 or 1 when cycle='G' or 0xffff
 */
static unsigned int smb_cycle(struct net_device *dev, unsigned char c)
{
	int i;
	unsigned int val=0;
	int symboltype=-1;
	unsigned char chip[4][2][3] = { /* signal every cycle as a triplet */
		{{1,1,1},  /* SCL */   /* START */
		 {1,0,0}}, /* SDA */
		{{1,1,1},  /* STOP */
		 {0,1,1}},
		{{0,1,0},  /* '1' or ACK */
		 {1,1,1}},
		{{0,1,0},  /* '0' or low */
		 {0,0,0}}
	};
	switch( c ) {
	case 'S': /* START is SDA = 1,0,0 while SCL high (1,1,1) */
		symboltype =  0;
		break;
	case 'O': /* STOP is SDA = 0,1,1 while SCL high (1,1,1) */
		symboltype =  1;
		break;
	case 'A':
	case '1':
	case 'R':
	case 'G': /* these are SDA = 1,1,1 (tristate) while a SCL pulse(0,1,0)*/
		symboltype =  2;
		break;
	case 'W':
	case '0': /* these are SDA = 0,0,0 (driven low) while a SCL pulse(0,1,0)*/
		symboltype =  3;
		break;
	case 'P':
		udelay(50); /* Pause: change to other value if desired */
		break;
	default:
		printk("*** intern error (SMB sequence wrong char '%c')\n",c);
	}
	if (symboltype >= 0) {
		for (i=0; i< 3; i++) {
			udelay(50);
			if ( (i==1) && (c == 'G')) /* for 'G' cycles, store SDA */
				val = z77_sda_in(Z077_BASE);
			z77_scl_out(Z077_BASE, chip[symboltype][0][i] );
			z77_sda_out(Z077_BASE, chip[symboltype][1][i] );
		}
	}
	return (c == 'G') ? val : 0xffff;
}




/*******************************************************************/
/** Read a byte from EEPROM
 *
 * \param dev		\IN net_device struct for this NIC
 * \param offset	\IN position of byte to read
 *
 * \return byte value at this offset in EEPROM
 * \brief
 *  Every cycle of the SMB access is coded in a descriptive string:
 *	S: Start    0:   0
 *	O: Stop     1:   1
 *	R: Read  	A:   Ack (SDA just set high, no check!)
 *	W: Write    P:   Pause 50 us
 *	G: Get bit (=reads SDA)
 *  Attention: this assumes that exactly one byte is read (8x'G' in a row)!
 */
static unsigned char z77_read_byte_data(struct net_device *dev,
										unsigned int offset )
{
	unsigned int i=0, j=7;
	unsigned char byte=0;
	unsigned char sequence[]="S1010000WA00000000AS1010000RAGGGGGGGGAO\0";

	for (i=0; i < 8; i++) /* insert offset in offset write phase above */
		sequence[10 + i] = (offset & (0x80>>i)) ? '1' : '0';
	i=0;
	while(sequence[i]) {
		if (sequence[i] == 'G') /* G: Get a bit (sda_in) */
			byte |= smb_cycle(dev, sequence[i++]) << (j--); /* assumes 8x'G'! */
		else  					/* other cycles: clocked out straightforward */
			smb_cycle(dev, sequence[i++]);
	}
	return byte;
}


/*******************************************************************/
/** Write a byte to EEPROM
 *
 * \param dev		\IN net_device struct for this NIC
 * \param offset	\IN position of byte to read
 * \param value 	\IN vlaue to write
 *
 * \return byte value at this offset in EEPROM
 *
 */
static void z77_write_byte_data(struct net_device *dev,
								unsigned int offset, unsigned char value )
{
	unsigned int i=0;
	unsigned char sequence[]="S1010000WA00000000A00000000AOP\0";

	/* insert offsets in 1st half of smb access (write-offset-phase) above*/
	for (i=0; i < 8; i++)
		sequence[10+i] = (offset & (0x80>>i)) ? '1' : '0';
	/* insert value in 2nd half of smb access (write-value-phase) above */
	for (i=0; i < 8; i++)
		sequence[19+i] = (value & (0x80>>i)) ? '1' : '0';
	i=0;
	while(sequence[i]) {
		smb_cycle(dev, sequence[i++]);
	}
}

/*******************************************************************/
/** reset the EEPROM
 *
 * \param dev		\IN net_device struct for this NIC
 *
 *
 */
static void	z77_reset_eeprom(struct net_device *dev )
{
	unsigned int i;
	for (i=0; i < 30; i++) {
		z77_scl_out(Z077_BASE, 0 );
		udelay(50);
		z77_scl_out(Z077_BASE, 1 );
		udelay(50);
	}
}
#endif


/*******************************************************************/
/** Initialize the IP core Registers
 *
 * \param dev			\IN net_device struct for this NIC
 * \param donegotiate	\IN nonzero if Autoneg. shall be performed
 *
 * \brief
 *  setup the necessary Registers like MODER, BDNUM (used on Z77 only)
 *  ans others such that the NIC can completely be restarted if needed.
 *
 * \return 0 or error code
 *
 */
static int chipset_init(struct net_device *dev, u32 donegotiate)
{
	u32 moder = 0;
	struct z77_private *np 	= netdev_priv(dev);
#if defined(MEN_P51X) || defined(CONFIG_X86)
	u32 mac_part = 0;
#endif
#if defined(CONFIG_MENEP05)
	struct device_node *z77_node = NULL;
	int* dev_index;
	int* dev_phy_addr;
	char* dev_mac_addr;
#endif

	Z77DBG(ETHT_MESSAGE_LVL1, "--> %s(%d)\n", __FUNCTION__, donegotiate);

	/* completely Reset MAC here, treat like powerup */
	z77_reset(dev);

	/* set MAC addr. if not done already */
	if( donegotiate==0 || !is_valid_ether_addr(dev->dev_addr) ) {
		/* -- set common MEN OUI part -- */
		dev->dev_addr[0] = 0x00;
		dev->dev_addr[1] = 0xc0;
		dev->dev_addr[2] = 0x3a;
#ifdef NIOS_II
		{
			u32 i = 0, val = 0;
			char tmp[3] = {0,0,0};
			unsigned char rawaddr[13]; /* MAC addr in ASCII: "00C03Axxyyzz" */
			u32 serialnumber = 0;
			if (SysParamGet("nmac0", rawaddr, 13)) {
				printk("*** error reading SysParam nmac0. Use default MAC Addr!\n");
				dev->dev_addr[3] = 0x41;
				dev->dev_addr[4] = 0x00;
				dev->dev_addr[5] = 0x00;

			} else {
				for (i = 0; i < 12; i+=2 ) {
					tmp[0] = rawaddr[i];
					tmp[1] = rawaddr[i+1];
					sscanf( tmp, "%x", &val );
					dev->dev_addr[i>>1] = val & 0xff;
				}
				SysParamGet("sernbr", rawaddr, 13);
				sscanf(rawaddr, "%d", &serialnumber);
				dev->dev_addr[4] = (serialnumber >> 8 ) & 0xFF;
				dev->dev_addr[5] = serialnumber & 0xFF;
			}
		}
#elif defined CONFIG_MENEM1
		{
			u32 i = 0, val = 0;
			char tmp[3] = {0,0,0};
			unsigned char rawaddr[13]; /* MAC addr in ASCII: "00C03A4xyyzz" */
			/* linux driver was used with EM01 first.. */
			if (SysParamGet("nmac0", rawaddr, 13)) {
				printk("*** error reading SysParam nmac0. Use default MAC\n");
				dev->dev_addr[3] = 0x42;
				dev->dev_addr[4] = 0x00;
				dev->dev_addr[5] = 0x01 ;
			} else {
				for (i = 0; i < 12; i+=2 ) {
					/* convert ASCII to bytes */
					tmp[0] = rawaddr[i];
					tmp[1] = rawaddr[i+1];
					sscanf( tmp, "%x", &val );
					dev->dev_addr[i>>1] = val & 0xff;
				}
				dev->dev_addr[3]++;	/* EM01 1st/2nd ETH = 0x40 / 0x41 */
			}
		}
#elif defined(CONFIG_MENEM9)
		{
			char buffer[20];

			dev->dev_addr[3] = 0xa4;

			if (SysParamGet("brd", buffer, sizeof(buffer)) == 0
				&& strncmp(buffer, "EM09A", sizeof("EM09A")) == 0
				&& np->instance < 2
				&& SysParamGet("nmac0", buffer, sizeof(buffer)) == 0) {

				dev->dev_addr[4] = (0x40 + 0x10 * np->instance)
								   | str2hexnum(buffer[9]);
				dev->dev_addr[5] = str2hexnum(buffer[10])<<4
								   | str2hexnum(buffer[11]);
			}
			else {
				printk(KERN_ERR "%s: *** Can't determine MAC address. "
					   "Use default MAC!\n", dev->name);
				dev->dev_addr[4] = 0x00;
				dev->dev_addr[5] = 0x01 + np->instance;
			}
		}
#elif defined(CONFIG_MENEP05)
		/* read phy addresses and mac addresses from fdt */
		while ((z77_node = of_find_node_by_type(z77_node, "z77_eth"))) {
			dev_index 	 = (int*) of_get_property( z77_node, MENEP05_Z77_INSTNAME, NULL );
			dev_mac_addr = (char*)of_get_property( z77_node, MENEP05_Z77_MACADR, NULL );
			dev_phy_addr = (int*) of_get_property( z77_node, MENEP05_Z77_PHYADR, NULL );
			if (dev_index == NULL) {
				printk(KERN_ERR "Error while parsing fdt for MEN_EP05: property %s not found in %s \n",
						MENEP05_Z77_INSTNAME, z77_node->name);
				continue;
			}
			/* is this our device? */
			if ( *dev_index == np->instance) {
	#ifndef NO_PHY
				/* store phy address */
				if ( dev_phy_addr ) {
					phyadr[np->instCount] = *dev_phy_addr;
					printk(KERN_INFO "PHY addr for %s: %d\n",
						   z77_node->name, *dev_phy_addr);
				} else {
					printk(KERN_ERR "Error: could not find PHY address in %s. Use default %d.\n",
							z77_node->name, phyadr[np->instCount]);
				}
	#endif
				/* store mac address */
				if ( dev_mac_addr ) {
					/* fixup the relevant parts */
					dev->dev_addr[3] = dev_mac_addr[3];
					dev->dev_addr[4] = dev_mac_addr[4];
					dev->dev_addr[5] = dev_mac_addr[5];
				} else {
					printk(KERN_ERR "Error: could not find MAC address in %s. Use default.\n",
							z77_node->name);

					dev->dev_addr[4] = 0x00;
					dev->dev_addr[5] = 0x01 + np->instance;
				}

				break;
			} /* instance is our device instance */
		}
#elif defined(MEN_MM1)  /* init SMB2 handle to read this instances EEPROM */
		/* read out this instances MAC */
		if (writemac)
		{
			/* clock the EEPROM like 30x to get it in default state (they have no Reset) */
			z77_reset_eeprom(dev);
			current->state = TASK_INTERRUPTIBLE;
			schedule_timeout(CONFIG_HZ/10); /* pause 100 ms */

			printk(KERN_ERR "programming  MAC EEPROM content:\n");
			z77_write_byte_data(dev, 0x01, 0x00 );
			current->state = TASK_INTERRUPTIBLE;
			schedule_timeout(CONFIG_HZ/10); /* pause 100 ms */

			z77_write_byte_data(dev, 0x02, 0xc0 );
			current->state = TASK_INTERRUPTIBLE;
			schedule_timeout(CONFIG_HZ/10); /* pause 100 ms */

			z77_write_byte_data(dev, 0x03, 0x3a );
			current->state = TASK_INTERRUPTIBLE;
			schedule_timeout(CONFIG_HZ/10); /* pause 100 ms */

			z77_write_byte_data(dev, 0x04, 0xae );
			current->state = TASK_INTERRUPTIBLE;
			schedule_timeout(CONFIG_HZ/10); /* pause 100 ms */

			z77_write_byte_data(dev, 0x05, 0x20 );
			current->state = TASK_INTERRUPTIBLE;
			schedule_timeout(CONFIG_HZ/10); /* pause 100 ms */

			z77_write_byte_data(dev, 0x06, 0x00 );
			current->state = TASK_INTERRUPTIBLE;
			schedule_timeout(CONFIG_HZ/10); /* pause 100 ms */
		} else {
			printk(KERN_ERR "dumping MAC EEPROM content\n");
			dev->dev_addr[0] = z77_read_byte_data(dev, 0x01 );
			dev->dev_addr[1] = z77_read_byte_data(dev, 0x02 );
			dev->dev_addr[2] = z77_read_byte_data(dev, 0x03 );
			dev->dev_addr[3] = z77_read_byte_data(dev, 0x04 );
			dev->dev_addr[4] = z77_read_byte_data(dev, 0x05 );
			dev->dev_addr[5] = z77_read_byte_data(dev, 0x06 );

			printk(KERN_INFO "%s: use EEprom MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
				   dev->name, dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
				   dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);
		}
#else
#  if defined(MEN_P51X) || (defined(CONFIG_X86) && !defined( MEN_MM1 ))
		/* assign the MAC found in EEPROM, consider instance count */
		switch (np->board) {
		case MEN_P511:
			mac_part = G_sernum + np->mac_offset + (np->instance * Z77_MAC_P511_IF2OFF);
			break;
		case MEN_P513:
			mac_part = G_sernum + np->mac_offset + (np->instance * Z77_MAC_P513_IF2OFF);
			break;
		case MEN_F11S: /* F218  & F11S has just 1 Z87 instance onboard */			
		case MEN_F218:
			mac_part = G_sernum + np->mac_offset;
			break;
		default:
			mac_part = 0;			
		}

		dev->dev_addr[3] = (mac_part & 0xFF0000) >> 16;
		dev->dev_addr[4] = (mac_part & 0x00FF00) >> 8;
		dev->dev_addr[5] = (mac_part & 0x0000FF);
#  else
		/* none of the above ? use generic MAC */
		dev->dev_addr[3] = 0xff;
		dev->dev_addr[4] = 0x00;
		dev->dev_addr[5] = 0x01 + np->instance;
#  endif
#endif

		/* set MAC Address in MAC_ADDR0, MAC_ADDR1 */
		Z77WRITE_D32( Z077_BASE, Z077_REG_MAC_ADDR0,
					  dev->dev_addr[2] << 24 | dev->dev_addr[3] << 16 | \
					  dev->dev_addr[4] << 8  | dev->dev_addr[5]);
		Z77WRITE_D32( Z077_BASE, Z077_REG_MAC_ADDR1,
					  dev->dev_addr[0] << 8 | dev->dev_addr[1] );

	} /* end if (donegotiate == 0) ... */

	np->mii_if.phy_id = phyadr[np->instCount];

#if defined(MEN_Z77_USE_OWN_PHYACCESS)
# ifndef NO_PHY
	if (donegotiate) {
		if (z77_do_autonegotiation(dev)) {
			printk(KERN_ERR "*** PHY mode setting / autonegotiate failed!\n");
			return(-ENODEV);
		}
	}
# endif
#endif

	moder = Z77READ_D32( Z077_BASE, Z077_REG_MODER );

#ifdef NO_PHY
    if( (moder & OETH_MODER_HD_AVAL) == 0 ) {
        moder = OETH_MODER_FULLD;
    }
#endif

	/* set MODER Register to meaningful values  */
	moder |= OETH_MODER_IFG | OETH_MODER_EXDFREN | OETH_MODER_CRCEN |	\
		OETH_MODER_BRO | OETH_MODER_PAD | OETH_MODER_RECSMALL;

	if ( ( (np->mii_if.full_duplex) || (strstr(mode, "FD"))) &&
		 (moder & OETH_MODER_HD_AVAL) ) {
		moder |= OETH_MODER_FULLD;
	}

	Z77WRITE_D32( Z077_BASE, Z077_REG_MODER, moder);

	/* set the packet length Register (remain at default) */
	Z77WRITE_D32( Z077_BASE, Z077_REG_PACKLEN, Z77_PACKLEN_DEFAULT );

	/* set BDNUM (const 64+64 at 16Z087) */
	Z77WRITE_D32( Z077_BASE, Z077_REG_TX_BDNUM, Z077_TBD_NUM );

	/* set management indices */
	np->nCurrTbd  	= 0;
	np->txIrq		= 0;

	Z77DBG(ETHT_MESSAGE_LVL1, "<-- %s()\n", __FUNCTION__);
	return(0);
}



/*******************************************************************/
/** The interrupt context Tx packet handler
 *
 * \brief Called from within main ISR context.
 *		  unmaps allocated PCI transfer memory region and updates
 *        stats info. No spin_lock here because main irq routine does
 *        local Eth core IRQ disabling already.
 *
 * \param dev		\IN net_device struct for this NIC
 *
 * \return -
 */
void z77_tx(struct net_device *dev)
{
	struct z77_private *np = netdev_priv(dev);

#ifdef CONFIG_PCI
	if (!nodma)
		pci_unmap_single(np->pdev, np->txBd[np->txIrq].hdlDma,
						 Z77_ETHBUF_SIZE,
						 PCI_DMA_TODEVICE);
#endif
	np->txIrq++;
	np->txIrq%= Z077_TBD_NUM;
	np->stats.tx_packets++;

	/* If we had stopped the queue due to a "tx full"
	 * condition, wake up the queue.
	 */
	if (netif_queue_stopped(dev))
		netif_wake_queue(dev);

}



/*****************************************************************************/
/** z77_pass_packet - packet passing function for one ETH frame
 *
 * \param dev		\IN net_device struct for this NIC
 * \param idx		\IN Rx BD index, 0..Z077_RBD_NUM
 *
 * \return 			-ENOMEM if packet squeeze or 0 on success
 */
static int z77_pass_packet( struct net_device *dev, unsigned int idx )
{

	struct z77_private *np = netdev_priv(dev);
	struct sk_buff *skb = NULL;
	u32 pkt_len = 0;
	u32 i = 0;
    char *buf = NULL;

	pkt_len	= Z077_GET_RBD_LEN( idx ) - LEN_CRC;
	pci_dma_sync_single_for_cpu( np->pdev,
								 np->rxBd[idx].hdlDma,
								 Z77_ETHBUF_SIZE, PCI_DMA_FROMDEVICE);

	skb = dev_alloc_skb( pkt_len + NET_IP_ALIGN );
	Z77DBG(ETHT_MESSAGE_LVL3, "z77_pass_packet[%d]: pktlen=%04x\n",
		   idx, pkt_len);

	if (skb) {
		skb->dev = dev;
		skb_reserve(skb, NET_IP_ALIGN); /* 16 byte align the IP fields. */
#if LINUX_VERSION_CODE > VERSION_CODE(2,6,23)
		skb_copy_to_linear_data(skb,
					(void*)(np->rxBd[idx].BdAddr | NO_CACHED_MEM),
					pkt_len);
#else
        eth_copy_and_sum(skb, (void*)(np->rxBd[idx].BdAddr | NO_CACHED_MEM),
						 pkt_len, 0);
#endif
		/* very verbose debugging on? then dump frame */
		if ( np->msg_enable >= ETHT_MESSAGE_LVL3 ) {
			buf = (char*)(np->rxBd[idx].BdAddr);
			Z77DBG(ETHT_MESSAGE_LVL3, "Frame:");
			for (i=0; i < pkt_len; i++) {
				if (!(i%16))
					Z77DBG(ETHT_MESSAGE_LVL3, "\n0x%03x: ", i);
				Z77DBG(ETHT_MESSAGE_LVL3, "%02x ", (unsigned char)(*buf++));
			}
			Z77DBG(ETHT_MESSAGE_LVL3, "\n");
		}

		skb_put(skb, pkt_len);

		skb->protocol = eth_type_trans (skb, dev);

		/* tell network stack... */
		netif_receive_skb(skb);

		dev->last_rx 		= jiffies;
		np->stats.rx_bytes += pkt_len;
		np->stats.rx_packets++;

		/* clean processed Rx BD nonempty Flag */
		if ( np->modCode == CHAMELEON_16Z077_ETH ) {
			Z077_SET_RBD_FLAG( idx, OETH_RX_BD_EMPTY | Z077_RBD_IRQ );
		} else {
			if ( idx < 32 )
				Z77WRITE_D32(Z077_BASE, Z077_REG_RXEMPTY0, 1<< idx );
			else
				Z77WRITE_D32(Z077_BASE, Z077_REG_RXEMPTY1, 1<< (idx-32));
		}
		return 0;
	} else {
		printk (KERN_WARNING "*** %s:Mem squeeze! drop packet\n",dev->name);
		np->stats.rx_dropped++;
		return -ENOMEM;
	}
}

/*******************************************************************/
/** The inverse routine to z77_open().
 *
 *  The network interfaces resources are deallocated here
 *
 * \param dev		\IN net_device struct for this NIC
 *
 * \return error code or 0 on sucess
 */
static int z77_close(struct net_device *dev)
{
	struct z77_private *np = netdev_priv(dev);

	Z77DBG( ETHT_MESSAGE_LVL1, "--> %s()\n", __FUNCTION__ );
	np->open_time = 0;
	np->flags &= ~(IFF_UP);
	netif_stop_queue(dev);

	/* disable IRQs */
	Z77WRITE_D32( Z077_BASE, Z077_REG_INT_MASK, 0 );
	/* clean spurious left IRQs */
	Z77WRITE_D32( Z077_BASE, Z077_REG_INT_SRC, 0x7f );

	/* stop receiving/transmitting */
	Z077_CLR_MODE_FLAG( OETH_MODER_RXEN | OETH_MODER_TXEN );

#if defined(MEN_Z77_USE_OWN_PHYACCESS)
	/* shutdown link detect timer */
	del_timer_sync(&np->timer);
#endif

#if LINUX_VERSION_CODE > VERSION_CODE(2,6,23)
	napi_disable(&np->napi);
#endif

#if !defined( CONFIG_PCI )
	/* release resources of Rx/Tx BDs */
	for (i = 0; i < Z077_TBD_NUM; i++ ) {
        kfree((void*)(np->txBd[i].BdAddr));
	}
	for (i = 0; i < Z077_RBD_NUM; i++ ) {
        kfree((void*)(np->rxBd[i].BdAddr));
    }
#endif

	/* disable Rx here. */
	free_irq(dev->irq, dev);

	Z77DBG( ETHT_MESSAGE_LVL1, "<-- %s()\n", __FUNCTION__ );

	return 0;
}


/*******************************************************************/
/** drop Rx buffers in case of BUSY interrupt (context: IRQ)
 *  in case we get a BUSY interrupt, throw away all packets
 *  (if this IRQ occurs we have some serious network flooding or
 *  CPU performance problem)
 */
void z77_drop_rxbuffers(struct net_device *dev)
{
	int i;
	struct z77_private *np = netdev_priv(dev);

	for ( i=0;i<Z077_RBD_NUM;i++ ) {
		/* clean processed Rx BD nonempty Flag */
		if ( np->modCode == CHAMELEON_16Z077_ETH ) {
			Z077_SET_RBD_FLAG( i, OETH_RX_BD_EMPTY | Z077_RBD_IRQ );
		} else {
			if ( i < 32 )
				Z77WRITE_D32(Z077_BASE, Z077_REG_RXEMPTY0, 1 << i );
			else
				Z77WRITE_D32(Z077_BASE, Z077_REG_RXEMPTY1, 1 << (i-32));
		}
	}
	printk(KERN_CRIT "MEN ETH BUSY IRQ, dropped Rx buffers\n");
}


/*******************************************************************/
/** central IRQ handler
 *
 * \param irq		\IN INTB interrupt
 * \param dev_id	\IN unique identifier
 * \param regs		\IN ancient stuff for register pushing/popping
 *
 * \return if IRQ was handled or not
 */
#if LINUX_VERSION_CODE >= VERSION_CODE(2,6,19)
static irqreturn_t z77_irq(int irq, void *dev_id)
#else
static irqreturn_t z77_irq(int irq, void *dev_id, struct pt_regs * regs)
#endif
{
	/* uses dev_id to store 'this' net_device */
	struct net_device *dev = (struct net_device *)dev_id;
	struct z77_private *np = netdev_priv(dev);
	int handled = 0;

	u32 status = Z77READ_D32( Z077_BASE, Z077_REG_INT_SRC );
    if (!status)
        goto out;	/* It wasnt me, ciao. */

	if (status & OETH_INT_RXF) { /* Got a packet. */
		Z077_DISABLE_IRQ( OETH_INT_RXF ); /* acknowledged/reenabled in NAPI poll routine */
#if LINUX_VERSION_CODE >= VERSION_CODE(2,6,30)
		napi_schedule(&np->napi);
# elif  (LINUX_VERSION_CODE >= VERSION_CODE(2,6,23)) && (LINUX_VERSION_CODE <= VERSION_CODE(2,6,28)) 
		netif_rx_schedule( dev, &np->napi );
# else
		netif_rx_schedule( dev );
#endif
	}

	if (status & OETH_INT_TXB) { /* Transmit complete. */
		z77_tx(dev);
	}

	if (status & OETH_INT_BUSY) { 	/* RX FIFO overrun ? */		
		np->gotBusy=1; 	/* clear after next NAPI poll round is done */
		Z77WRITE_D32(Z077_BASE, Z077_REG_INT_SRC, status & ~OETH_INT_RXE );
	}

	if (status & OETH_INT_TXE) {  	/* handle Tx Error */
		z77_tx_err(dev);
	}
	
	if (status & OETH_INT_RXE) {	/* handle Rx Error */
		z77_rx_err(dev);
	}

	/* acknowledge all IRQs except RXF (leave it asserted until current NAPI poll cycle is finished) */
	Z77WRITE_D32(Z077_BASE, Z077_REG_INT_SRC, status & ~OETH_INT_RXF );

	handled = 1;
 out:
	return IRQ_RETVAL(handled);
}

/*****************************************************************************/
/** Get the current statistics.
 *
 * \param dev		\IN net_device struct for this NIC
 * \return pointer to device status struct
 */
static struct net_device_stats *z77_get_stats(struct net_device *dev)
{
	struct z77_private *np = netdev_priv(dev);
	/* Update the statistics from the device registers. */
	return &np->stats;
}

#if !defined(MEN_Z77_USE_OWN_PHYACCESS)
/*****************************************************************************/
/** Check link status changes reported from phy device
 *
 * \param dev		\IN net_device struct for this NIC
 *
 */
static void z77_handle_link_change(struct net_device *dev)
{
	struct z77_private *np = netdev_priv(dev);
	struct phy_device *phydev = np->phy_dev;
	unsigned long flags;

	int status_change = 0;
	spin_lock_irqsave(&np->lock, flags);

	if (phydev->link) {
		if (np->duplex != phydev->duplex) {
			if (phydev->duplex)
				Z077_SET_MODE_FLAG(OETH_MODER_FULLD);
			else
				Z077_CLR_MODE_FLAG(OETH_MODER_FULLD);

			/* store updated value */
			status_change = 1;
			np->duplex = phydev->duplex;
		}

		if (np->speed != phydev->speed) {
			status_change = 1;
			switch (phydev->speed) {
			case 100:
			case 10:
				/* printk(KERN_WARNING " Speed = %d\n", phydev->speed); */
				break;
			default:
				printk(KERN_WARNING
				       "%s: *** Speed (%d) is not 10/100\n", dev->name,
				       phydev->speed);
				break;
			}
			/* store updated value */
			status_change = 1;
			np->speed = phydev->speed;
		}
	}

	if (status_change) {
		if (phydev->link) {
			printk(KERN_INFO "%s: link up (%d/%s)\n",
			       dev->name, phydev->speed,
			       DUPLEX_FULL == phydev->duplex ? "Full" : "Half");

			z77_reset( dev );
			Z077_SET_MODE_FLAG( OETH_MODER_RXEN | OETH_MODER_TXEN );
			np->nCurrTbd = 0;
		}
		else
			printk(KERN_INFO "%s: link down\n", dev->name);
	}

	spin_unlock_irqrestore (&np->lock, flags);
}


/*****************************************************************************/
/** dummy for MDIO phy reset
 *
 * \param bus 		\IN mii_bus struct attached to this NIC
 *
 */
static int z77_mii_bus_reset(struct mii_bus *bus)
{
	return 0;
}


/*****************************************************************************/
/** Probe PHY devices on give MII bus
 *
 * \param dev		\IN net_device struct for this NIC
 * \return pointer to device status struct
 */
static int z77_mii_probe(struct net_device *dev)
{

	struct z77_private *np = netdev_priv(dev);
	struct phy_device *phydev = NULL;
	int phy_addr;

	/* find the first phy */
	for (phy_addr = 0; phy_addr < PHY_MAX_ADDR; phy_addr++) {
		if (np->mii_bus->phy_map[phy_addr]) {
			phydev = np->mii_bus->phy_map[phy_addr];
			break;
		}
	}

	if (!phydev) {
		printk(KERN_ERR "%s: no PHY found\n", dev->name);
		return -ENODEV;
	}

	/* attach the mac to the phy */
	phydev = phy_connect(dev, dev_name(&phydev->dev),
						 &z77_handle_link_change, 0,
						 PHY_INTERFACE_MODE_MII);

	if (IS_ERR(phydev)) {
		printk(KERN_ERR "%s: Could not attach to PHY\n", dev->name);
		return PTR_ERR(phydev);
	}

	/* mask with MAC supported features - for MEN Z87 only basic features */
	phydev->supported &= PHY_BASIC_FEATURES;

	phydev->supported |= SUPPORTED_Asym_Pause | SUPPORTED_Pause;
	phydev->advertising = phydev->supported;

	np->speed 	= 0;
	np->duplex 	= -1;
	np->phy_dev = phydev;

	return 0;
}

/*****************************************************************************/
/** init MII bus from phy layer
 *
 * \param dev		\IN net_device struct for this NIC
 * \return 0 on success or negative error
 */
static int z77_mii_init(struct net_device *dev)
{
	int err, i;
	struct z77_private *np = netdev_priv(dev);

	np->mii_bus = mdiobus_alloc();
	if (np->mii_bus == NULL)
		return -ENOMEM;

	np->mii_bus->priv	= (void*)dev;
	np->mii_bus->name 	= "z77_mii_bus";
	np->mii_bus->read 	= &z77_mii_bus_read;
	np->mii_bus->write 	= &z77_mii_bus_write;
	np->mii_bus->reset 	= &z77_mii_bus_reset;
	snprintf(np->mii_bus->id, MII_BUS_ID_SIZE, "men_z77_%d", np->instCount);

	np->mii_bus->irq = kmalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);
	if (!np->mii_bus->irq) {
		err = -ENOMEM;
		goto err_out;
	}

	for (i = 0; i < PHY_MAX_ADDR; i++)
		np->mii_bus->irq[i] = PHY_POLL;

	platform_set_drvdata(np->dev, np->mii_bus);

	if (mdiobus_register(np->mii_bus)) {
		err = -ENXIO;
		goto err_out_free_mdio_irq;
	}

	if (z77_mii_probe(np->dev) != 0) {
		err = -ENXIO;
		goto err_out_unregister_bus;
	}

	return 0;

err_out_unregister_bus:
	mdiobus_unregister(np->mii_bus);
err_out_free_mdio_irq:
	kfree(np->mii_bus->irq);
err_out:
	mdiobus_free(np->mii_bus);
	return err;
}
#endif /* !defined(MEN_Z77_USE_OWN_PHYACCESS) */

/*****************************************************************************/
/** z77_retrieve_fpga_name - get FPGA name of 'this' Z87
 *
 *  called to obtain the FPGA filename so special cases like F218 and P511 can
 *  be handled.
 *
 * \param dev		\IN network device
 * \param chu		\IN chameleon unit found
 *
 * \return 			0 on success or -1 on error
 *
 */
static int z77_retrieve_fpga_name( struct net_device *dev, CHAMELEON_UNIT_T *chamUnit )
{
	u32 value32 		= 0;
	int retval 			= 0;
	CHAM_FUNCTBL 		chamFctTable;
    CHAMELEONV2_TABLE   table;
    CHAMELEONV2_HANDLE 	*chamHdl=NULL;
	struct z77_private  *np = NULL;

	if ( (dev == NULL ) || ( chamUnit == NULL ) )
		return(-1);

	np = netdev_priv(dev);

	/*
	 * we have no direct handle to FPGA table from CHAMELEON_UNIT_T,
	 * so set up a temporary chamFctTable and call TableIdent().
	 */
	OSS_Init(MEN_Z77_DRV_NAME, &G_osh);

	/* check if we are on IO or memory mapped FPGA table */
    pci_read_config_dword( chamUnit->pdev, PCI_BASE_ADDRESS_0, &value32 );

    if ( value32 & PCI_BASE_ADDRESS_SPACE_IO ) { /* io mapped */
#ifndef CONFIG_PPC
    	CHAM_InitIo( &chamFctTable );
#else	/* for PPCs, no I/O Mapping is supported; init as Mem mapped */
		CHAM_InitMem( &chamFctTable );
#endif
    } else {
    	CHAM_InitMem( &chamFctTable );
    }
	retval = chamFctTable.InitPci( G_osh, chamUnit->pdev->bus->number,
								   chamUnit->pdev->devfn>>3, 0, &chamHdl );
	if (!retval) {
		chamFctTable.TableIdent( chamHdl, 0, &table );
		Z77DBG( ETHT_MESSAGE_LVL2, MEN_Z77_DRV_NAME " Z87 instance found on FPGA '%s'\n", table.file );

		if (!(strncmp(table.file, "F011", IC_BOARD_NAME_LEN ))) {
			np->mac_offset  = Z77_MAC_OFFS_F11S;
			np->board		= MEN_F11S;
		}

		if (!(strncmp(table.file, "F218", IC_BOARD_NAME_LEN ))) {
			np->mac_offset  = Z77_MAC_OFFS_F218;
			np->board		= MEN_F218;
		}

		if (!(strncmp(table.file, "P511", IC_BOARD_NAME_LEN ))) {
			np->mac_offset 	= Z77_MAC_OFFS_P511;
			np->board		= MEN_P511;
		}

		if (!(strncmp(table.file, "P513", IC_BOARD_NAME_LEN ))) {
			np->mac_offset 	= Z77_MAC_OFFS_P513;
			np->board		= MEN_P513;
		}

		OSS_Exit( &G_osh );
	} else {
		printk( KERN_INFO MEN_Z77_DRV_NAME " *** couldnt find FPGA table, exit. (retval = %x)\n", retval );
		OSS_Exit( &G_osh );
		return(-1);
	}
	return 0;
}



/*****************************************************************************/
/** men_16z077_probe - PNP function for ETH
 *
 * This gets called when the chameleon PNP subsystem starts and
 * is called for each Eth unit. probe() is called when a 16Z077/87 instance
 * was found
 *
 * \param chu		\IN wdt unit found
 * \return 			0 on success or negative linux error number
 *
 */
int men_16z077_probe( CHAMELEON_UNIT_T *chu )
{

	u32 phys_addr 			= 0;
	u32 ddr_addr 			= 0;
	u32 ddr_len 			= 0;
	void* ddr_virt 			= 0;
	struct net_device *dev 	= NULL;
	struct z77_private *np 	= NULL;

	printk(KERN_INFO MEN_Z77_DRV_NAME " passed PHY mode(s): '%s'\n", mode);

	dev = alloc_etherdev(sizeof(struct z77_private));
	if (!dev)
		return -ENOMEM;

#ifdef CONFIG_PCI
	if (pci_set_dma_mask(chu->pdev, Z87_BIT_MASK_32BIT) < 0) {
 		printk(KERN_ERR MEN_Z77_DRV_NAME " *** DMA not suitable supported, exiting.\n");
		goto err_free_reg;
	}

	if (pci_enable_device(chu->pdev)) {
 		printk(KERN_ERR MEN_Z77_DRV_NAME "*** Error during pci_enable_device.\n");
		return -ENODEV;
	}

	/* enable bus mastering */
	pci_set_master(chu->pdev);
	phys_addr = pci_resource_start(chu->pdev, chu->bar) + chu->offset;

	if (nodma) {
		/*
		 * nodma=1 was passed: check if we are a P511 & remap PCI DDR on P511.
		 * ensure that we are really servicing a P511. Check if:
		 *  1) chameleon FPGA PCI bus number is !=0 (all ESM FPGAs are on bus 0)
		 *  2) PCI Memory same chamel.group at BAR1 with size 0x2000000 (32M)
		 */

		/* TODO: seek Z043 of same chameleonV2 group! current offset is 0 */
		ddr_addr = pci_resource_start(chu->pdev, P511_SDRAM_BAR) + 0;
		ddr_len  = pci_resource_len(  chu->pdev,  P511_SDRAM_BAR );

		if ((ddr_len == P511_BAR1_SDRAMSIZE) && ( chu->pdev->bus->number)) {
			ddr_virt = ioremap_nocache( ddr_addr, ddr_len  );
			printk(KERN_INFO "P511: BAR1 (phys 0x%08x len 0x%x remap to %p)\n",
				   ddr_addr, ddr_len, ddr_virt );
		} else {
			printk(KERN_ERR "*** nodma=1 but device seems no P511!\n");
			return -ENODEV;
		}
	}

	/* base_addr is complete range from MODER to Rx Ring end  */
    dev->base_addr=(unsigned long)ioremap_nocache(phys_addr, (u32)Z77_CFGREG_SIZE );
#else
    /* offset contains absolute address */
    phys_addr = NO_CACHED_MEM | chu->offset;
    dev->base_addr = phys_addr;
#endif
	dev->irq       = chu->irq;

	if( dev_alloc_name( dev, "eth%d") < 0)
		printk("*** warning: couldnt retrieve a name for Z77\n");

	/* setup the phy Info within the private z77_private struct */
	np = netdev_priv(dev);

#ifdef CONFIG_PCI
	np->pdev = chu->pdev;
	pci_set_drvdata(chu->pdev, dev);
#endif

#if LINUX_VERSION_CODE > VERSION_CODE(2,6,23)
    netif_napi_add(dev, &np->napi, z77_poll, Z077_WEIGHT);
    np->dev = dev;
#endif

	/*
	 * store instance of 'this' Z87 to assign correct PHY address later
	 * attention: if 2 Ethernets are in 2 FPGAs every chu->instance is 0!
	 * therefore also add instance count
	 */
	np->instance  = chu->instance;
	np->instCount = G_globalInstanceCount;
	G_globalInstanceCount++;

	/* link change detect timer polls every 0.5 s */
	np->timer_offset = CONFIG_HZ/2;

#if defined(Z77_USE_VLAN_TAGGING)
	dev->features |= Z87_VLAN_FEATURES;
	/* on older kernels also assign the register functions here */
# if  !defined(Z77_VLAN_OPS_CHANGE)
	dev->vlan_rx_register = z77_vlan_rx_register;
#  if LINUX_VERSION_CODE < VERSION_CODE(3,0,0) 
	dev->vlan_rx_kill_vid = z77_vlan_rx_kill_vid;
#  endif
# endif
#endif

	/* pass initial debug level */
	np->msg_enable = (dbglvl > Z77_MAX_MSGLVL) ? Z77_MAX_MSGLVL : dbglvl;

	printk(KERN_INFO MEN_Z77_DRV_NAME " initial debug level %d\n", np->msg_enable );

	/* Init bdBase and IP core related stuff depending on found core */
	np->modCode = chu->modCode;

	if (np->modCode == CHAMELEON_16Z077_ETH ) {
		/* old OpenCore: BDs are placed at <FPGABase>+[0x400..0x800] */
		np->bdBase 		= 	dev->base_addr + Z077_BD_OFFS;
		np->tbdOff		=	0;
		np->rbdOff		=	Z077_TBD_NUM;
		np->bdOff		=	Z077_BD_AREA_SIZE;
		strncpy(cardname, "16Z077", sizeof(cardname));

	} else { /* New Cores */

      if (!(np->bdBase = (unsigned long)kmalloc( Z077_BD_AREA_SIZE+Z077_BDALIGN, GFP_KERNEL))) {
			printk( KERN_ERR "*** %s: kmalloc bdBase failed!\n", __FUNCTION__);
			goto err_free_reg;
		}
		#ifdef NIOS_II
		/* Z087 need bdBase aligned */
		np->bdBase = (((np->bdBase & ~(Z077_BDALIGN-1))
		               + Z077_BDALIGN) | NO_CACHED_MEM);
        #endif
		np->tbdOff	= Z077_TBD_NUM;
		np->rbdOff	= 0;
		np->bdOff	= 0;

		/* clean BD Area */
		memset((void*)np->bdBase, 0, Z077_BD_AREA_SIZE );
		strncpy(cardname, "16Z087", sizeof(cardname));

		/* and tell 16Z087 where the BDs start, without virtual mm offset */
		Z77WRITE_D32( Z077_BASE, Z077_REG_BDSTART, np->bdBase & ~VIRTUAL_MEM_OFFSET );
	}

	if (nodma) {
		np->ddr_len		= ddr_len;
		np->ddr_addr	= ddr_addr;
		np->ddr_virt	= ddr_virt;
	}

	printk(KERN_INFO MEN_Z77_DRV_NAME " found %s (phys 0x%08x irq 0x%x base addr 0x%08x )\n",
		   cardname, (u32)phys_addr, chu->irq, (u32)(dev->base_addr));

	/* ok so far, store dev in Chameleon units driver_data for removal */
	chu->driver_data = (void*)dev;

	/* Check if its a 'new' Z87 (better would be an own IP core though..) */
	if ( Z77READ_D32( dev->base_addr, Z077_REG_RXBDSTAT ) & REG_RXSTAT_REV ) {
		np->coreRev = 1;
		printk(KERN_INFO "Z077_REG_RXSTAT[6]=1: new Revision\n");
	} else {
		np->coreRev = 0;
	}

	/* Some Cards need special MAC number assignment.
	 * retrieve 'real' board name/number from its ident Table and
	 * assign their base MAC to the z7_private struct
	 */
	if (z77_retrieve_fpga_name( dev, chu ) < 0)

	/* force new interrupt behavior */
	Z77WRITE_D32( dev->base_addr, Z077_REG_COREREV, Z77READ_D32(dev->base_addr,
	              Z077_REG_COREREV) | REG_COREREV_IRQNEWEN );

	if( !(Z77READ_D32(dev->base_addr, Z077_REG_COREREV)
           & REG_COREREV_IRQNEWEN) ) {
		printk(KERN_WARNING "%s: Couldn't set to new IRQ behavior\n", __func__);
	}

#if defined(MEN_Z77_USE_OWN_PHYACCESS)
	/* set up timer to poll for link state changes */
	init_timer(&np->timer);
	np->timer.expires = jiffies + np->timer_offset;
	np->timer.data 	= (unsigned long)dev;
	np->timer.function 	= z77_timerfunc;
#endif
	
	/* init the process context work queue function to restart Z77 */
	INIT_WORK(&np->reset_task, z77_reset_task);

	/* All members set up, start probing PHY with driver builtin functions */
	if (probe_z77(dev) == 0) {
#if !defined(MEN_Z77_USE_OWN_PHYACCESS)
		printk(KERN_INFO "probing for phy devices..");
		if (!z77_mii_init(dev)) {
			printk(KERN_INFO "..probing succeeded.\n");
		}
#endif
		if (register_netdev(dev) == 0) {
			/* Create linkstate sysfs file */
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,21) )
			if (device_create_file(&dev->dev, &dev_attr_linkstate))
				dev_err(&dev->dev, "Error creating sysfs file\n");
#endif
		}
		return 0;
	}

err_free_reg:
	/* arrived here, we failed */
#ifdef CONFIG_PCI
	pci_set_drvdata(chu->pdev, NULL);
#endif
	free_netdev(dev);
	printk("*** men_16z077_probe failed\n");
	cleanup_card(dev);
	return -ENXIO;
}

/*******************************************************************/
/** PNP function to remove the unit driver
 *
 *  The netdev struct is kept in struct CHAMELEON_UNIT_Ts private driver
 *  data pointer so any number of found netdevs can be freed.
 *
 * \param chu		\IN IP core unit to remove
 *
 * \return 0 on success or negative linux error number
 */
static int men_16z077_remove( CHAMELEON_UNIT_T *chu )
{

	struct net_device *dev = (struct net_device *)chu->driver_data;
	struct z77_private *np = netdev_priv(dev);
    Z77DBG( ETHT_MESSAGE_LVL2, "--> men_16z077_remove" );

	if (nodma)
		iounmap(np->ddr_virt );
#ifndef MEN_Z77_USE_OWN_PHYACCESS
	if (np->mii_bus)
		mdiobus_unregister(np->mii_bus);

	if (np->mii_bus->irq)
		kfree(np->mii_bus->irq);
#endif

	/* remove the work queue */
	cancel_work_sync(&np->reset_task);

	unregister_netdev(dev);	/* say goodbye to the kernel */

	kfree((void*)np->bdBase);
	return 0;
}

static u16 G_modCodeArr[] = {
	CHAMELEON_16Z077_ETH,
	CHAMELEON_16Z087_ETH,
	CHAMELEON_MODCODE_END
};

static CHAMELEON_DRIVER_T G_driver = {
	.name		=	"z077-eth",
	.modCodeArr = 	G_modCodeArr,
	.probe		=	men_16z077_probe,
	.remove		= 	men_16z077_remove
};


/*******************************************************************/
/** ioctl function - interface to applications
 *
 * \param dev		\IN net_device struct for this NIC
 * \param ifr       \IN interface request struct
 * \param cmd       \IN command
 *
 * \return 0 on success or negative linux error number
 */
static int z77_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct mii_ioctl_data *data = if_mii(ifr);
	int value;
	unsigned char phyAddr;
	struct z77_private *np = netdev_priv(dev);

	switch (cmd) {
	case SIOCGMIIPHY:
		phyAddr = np->mii_if.phy_id;
		data->phy_id = phyAddr;
		break;
	case SIOCGMIIREG:
		value = z77_mdio_read(dev, data->phy_id, data->reg_num);
		data->val_out = value;
		break;
	case SIOCSMIIREG:
		z77_mdio_write(dev, data->phy_id, data->reg_num, data->val_in);
		break;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}


/********************************************************************/
/** drivers cleanup routine
 *
 */
static void __exit men_16z077_cleanup(void)
{
	men_chameleon_unregister_driver( &G_driver );

#if defined(MEN_MM1)  /* deinit SMB2 lib for EEPROM readout */
	printk("TODO SMB2 deinit ? \n");
#endif

}

/*******************************************************************/
/** module init function
 *
 *  We are a chameleon unit driver rather than a true standalone
 *  Ethernet driver, so all Z77 detection, negotiation and stuff is
 *  done at its probe() routine
 *
 * \return		0 or error code from men_chameleon_register_driver
 */
static int __init men_16z077_init(void)
{

	int i=0;
#if defined(MEN_P51X) || (defined(CONFIG_X86) && !defined( MEN_MM1 ))
	int board = 0;
	char brdname[6];  /* "F011S_" */
	unsigned int sernum=0;  /* "F011S_" */
	struct i2c_adapter *adap 		= NULL;
	struct i2c_board_info i2cinfo;
	struct i2c_client  *client 		= NULL;
	int j=0, k=0;

	memset(&i2cinfo, 0, sizeof(struct i2c_board_info));
	i2cinfo.addr = MEN_F1x_EEID_SMB_ADR;
	i2cinfo.flags = I2C_CLASS_HWMON;
	strncpy(i2cinfo.type, "EEP", 3);

#endif
	printk(KERN_INFO MEN_Z77_DRV_NAME "\n");
#if defined(MEN_Z77_USE_OWN_PHYACCESS)
	printk(KERN_INFO MEN_Z77_DRV_NAME " using builtin PHY access functions.\n");
#else
	printk(KERN_INFO MEN_Z77_DRV_NAME " using phy driver platform functions.\n");
#endif
#if defined(Z77_USE_VLAN_TAGGING)
	printk(KERN_INFO MEN_Z77_DRV_NAME " VLAN tag support enabled.\n");
#endif
	printk(KERN_INFO MEN_Z77_DRV_NAME " version %s\n", version);

	/* sanity checks, parse phy modes */
	if (( nodma ) && (nodma != 1)) {
		printk(KERN_ERR "*** invalid nodma parameter, must be 0 or 1 !\n");
		goto errout;
	}

    for (i = 0; i < NR_ETH_CORES_MAX; i++ ) {
		if ( (phyadr[i] < 0) || ((phyadr[i] > PHY_MAX_ADR))) {
			printk(KERN_ERR "*** invalid phyadr[%d] = %d, must be 0..31 !\n",
				   i, phyadr[i] );
			goto errout;
		}
	}
	z77_parse_mode(strlen(mode), mode);

#if defined(MEN_P51X) || (defined(CONFIG_X86) && !defined( MEN_MM1 ))
	/* if F1x or P51x board: try detecting the ID EEPROM and read out name & serial nr.
	   we try finding device address 0x57 (0xAE in 8bit notation).
	   currently we have only F11S with an FPGA interface, but be prepared for sidecards
	   or PMCs.	*/
	for (i=0; i<I2C_MAX_ADAP_CNT; i++) {
		adap = i2c_get_adapter(i);
		if (adap != NULL) {
			memset( brdname, 0x0, sizeof(brdname));
			sernum = 0;
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25) )
			/* client = i2c_new_dummy( adap, MEN_F1x_EEID_SMB_ADR ); */
			client = i2c_new_device( adap, &i2cinfo );
#else
			client = i2c_get_client(0, I2C_CLASS_HWMON, client);
#endif
			if (client) {
				/* EEPROM @ 0x57 found, 9. char is 'F' -> looks very much like board ID */
				for ( j=ID_EE_NAME_OFF; j < (ID_EE_NAME_OFF + ID_EE_NAME_LEN); j++ ) {
					brdname[j-ID_EE_NAME_OFF] = i2c_smbus_read_byte_data(client, j);
				}
				for ( j=ID_EE_SERN_OFF + ID_EE_SERN_LEN,k=0; j > ID_EE_SERN_OFF; j--,k++ )
					sernum |= (i2c_smbus_read_byte_data(client, j)) << (k*8);

				/* found F1x or P51x board ? */
				if (brdname[0] == 'F') {
					if ( brdname[1] == '0')
						board = (10*(brdname[2]-'0') + (brdname[3]-'0'));
					else
						board = (10*(brdname[1]-'0') + (brdname[2]-'0'));
				}

				if (brdname[0] == 'P') /* always '511' or '513' */
						board = (100*(brdname[1]-'0') + (10 * brdname[2]-'0') + (brdname[3]-'0'));

				if (board > 0) {
					/* unregister & put back claimed dummy device and client, */
					/* or module unload/ reload leads to kernel oops (adapter still busy).*/
					i2c_unregister_device( client );
					i2c_put_adapter( adap );
					break;
				}
				i2c_unregister_device( client );
			}
			i2c_put_adapter( adap );
		}
	}

	/* no meaningful data or nothing found ? complain but use default MAC */
	if ( board == 0 ) {
		printk(KERN_WARNING MEN_Z77_DRV_NAME " *** no board ident EEPROM with serial# found. Using default.\n",
			   board, sernum);
	} else {
		printk(KERN_INFO MEN_Z77_DRV_NAME " Found ID EEPROM: board: %d ser# %ld\n",
			   board, sernum);
		/* set the global macadr[6] least 3 byte according to Ethernet numbers document */
		G_sernum = sernum;
	}
#endif

	if (serialnr > 0)
		G_sernum = serialnr;

	/* men_chameleon_register_driver returns count of found instances */
	if (!men_chameleon_register_driver( &G_driver ))
		return -ENODEV;
	else
		return 0;

 errout:
	return -EINVAL;
}

module_init(men_16z077_init);
module_exit(men_16z077_cleanup);

MODULE_LICENSE( "GPL" );
MODULE_DESCRIPTION( "MEN Ethernet IP core unit driver" );
MODULE_AUTHOR("thomas.schnuerer@men.de");
