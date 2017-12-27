/* 
 * This file is part of the Nautilus AeroKernel developed
 * by the Hobbes and V3VEE Projects with funding from the 
 * United States National  Science Foundation and the Department of Energy.  
 *
 * The V3VEE Project is a joint project between Northwestern University
 * and the University of New Mexico.  The Hobbes Project is a collaboration
 * led by Sandia National Laboratories that includes several national 
 * laboratories and universities. You can find out more at:
 * http://www.v3vee.org  and
 * http://xtack.sandia.gov/hobbes
 *
 * Copyright (c) 2017, Panitan Wongse-ammat
 * Copyright (c) 2017, Peter Dinda
 * Copyright (c) 2017, The V3VEE Project  <http://www.v3vee.org> 
 *                     The Hobbes Project <http://xstack.sandia.gov/hobbes>
 * All rights reserved.
 *
 * Authors: Panitan Wongse-ammat <Panitan.W@u.northwesttern.edu>
 *          Marc Warrior <warrior@u.northwestern.edu>
 *          Galen Lansbury <galenlansbury2017@u.northwestern.edu>
 *          Peter Dinda <pdinda@northwestern.edu>
 *
 * This is free software.  You are permitted to use,
 * redistribute, and modify it as specified in the file "LICENSE.txt".
 */

#ifndef __E1000E_PCI
#define __E1000E_PCI

// Constant variables
// These variables are configurable, and they will change the number of descriptors.
// The number of descriptors is always a multiple of eight.
// both tx_dsc_count and rx_dsc_count should be multiple of 16.
#define TX_DSC_COUNT          16      
#define TX_BLOCKSIZE          256      // bytes available per DMA block
#define RX_DSC_COUNT          16       // equal to DMA block count
#define RX_BLOCKSIZE          256      // bytes available per DMA block
#define RESTART_DELAY         5        // usec = 5 us 

// After this line, PLEASE DO NOT CHANGE ANYTHING.
// transmission unit
// Data sheet from page 36
// 16384 bytes is from the maximum packet buffer size that e1000 can receive.
// 16288 bytes is the maximum packet size that e1000 can send in theory.
// Ethernet standard MTU is 1500 bytes.
#define MAX_TU                    16384    // maximum transmission unit 
#define MIN_TU                    48       // minimum transmission unit 
#define MAC_LEN                   6        // the length of the mac address

// PCI CONFIG SPACE ************************************
#define INTEL_VENDOR_ID               0x8086
#define E1000E_DEVICE_ID              0x10D3
#define E1000E_PCI_CMD_OFFSET         0x4    // Device Control - RW
#define E1000E_PCI_STATUS_OFFSET      0x6    // Device Status - RO

// PCI command register
#define E1000E_PCI_CMD_IO_ACCESS_EN   1       // io access enable
#define E1000E_PCI_CMD_MEM_ACCESS_EN  (1<<1)  // memory access enable
#define E1000E_PCI_CMD_LANRW_EN       (1<<2)  // enable mastering lan r/w
#define E1000E_PCI_CMD_INT_DISABLE    (1<<10) // legacy interrupt disable when set

// PCI status register
#define E1000E_PCI_STATUS_INT         (1<<3)

// REGISTER OFFSETS ************************************
#define E1000E_CTRL_OFFSET    0x00000
#define E1000E_STATUS_OFFSET  0x00008
#define E1000E_GCR_OFFSET     0x05B00
// flow control address
#define E1000E_FCAL_OFFSET    0x00028
#define E1000E_FCAH_OFFSET    0x0002C
#define E1000E_FCT_OFFSET     0x00030

// transmit
#define E1000E_TDBAL_OFFSET   0x3800   // transmit descriptor base address low check
#define E1000E_TDBAH_OFFSET   0x3804   // transmit descriptor base address high check
#define E1000E_TDLEN_OFFSET   0x3808   // transmit descriptor list length check
#define E1000E_TDH_OFFSET     0x3810   // transmit descriptor head check
#define E1000E_TDT_OFFSET     0x3818   // transmit descriptor tail check
#define E1000E_TCTL_OFFSET    0x0400   // transmit control check check
#define E1000E_TIPG_OFFSET    0x0410   // transmit interpacket gap check
#define E1000E_TXDCTL_OFFSET  0x03828  // transmit descriptor control r/w check

// receive
#define E1000E_RAL_OFFSET     0x5400   // receive address low check
#define E1000E_RAH_OFFSET     0x5404   // receive address high check
#define E1000E_RDBAL_OFFSET   0x2800   // rx descriptor base address low check
#define E1000E_RDBAH_OFFSET   0x2804   // rx descriptor base address high check
#define E1000E_RDLEN_OFFSET   0x2808   // receive descriptor list length check
#define E1000E_RDH_OFFSET     0x2810   // receive descriptor head check
#define E1000E_RDT_OFFSET     0x2818   // receive descriptor tail check
#define E1000E_RCTL_OFFSET    0x0100   // receive control check
#define E1000E_RDTR_OFFSET_NEW 0x2820
#define E1000E_RDTR_OFFSET_ALIAS 0x0108 // receive delay timer check
#define E1000E_RSRPD_OFFSET   0x02C00  // rx small packet detect interrupt r/w
#define E1000E_RXDCTL_OFFSET  0x2828   // receive descriptor control
#define E1000E_RADV_OFFSET    0x282C   // receive interrupt absolute delay timer

// statistics error
#define E1000E_CRCERRS_OFFSET 0x04000  // crc error count
//// missed pkt count (insufficient receive fifo space)
#define E1000E_MPC_OFFSET     0x04010

#define E1000E_TPT_OFFSET     0x40D4   // total package transmit
#define E1000E_TPR_OFFSET     0x40D0   // total package receive
#define E1000E_TORL_OFFSET    0x040C0  // total octet receive low
#define E1000E_TORH_OFFSET    0x040C8  // total octet receive high
#define E1000E_TOTL_OFFSET    0x040C8  // total octet transmit low 
#define E1000E_TOTH_OFFSET    0x040CC  // total octet transmit high

#define E1000E_RXERRC_OFFSET  0x0400C  // rx error count
#define E1000E_RNBC_OFFSET    0x040A0  // receive no buffer count
#define E1000E_GPRC_OFFSET    0x04074  // good packets receive count
#define E1000E_GPTC_OFFSET    0x04080  // good packets transmit count
#define E1000E_COLC_OFFSET    0x04028  // collision count
#define E1000E_RUC_OFFSET     0x040A4  // receive under size count
#define E1000E_ROC_OFFSET     0x040AC  // receive over size count

// 4 interrupt register offset
#define E1000E_ICR_OFFSET     0x000C0  /* interrupt cause read register check*/
#define E1000E_ICS_OFFSET     0x000C8  /* interrupt cause set register */
#define E1000E_IMS_OFFSET     0x000D0  /* interrupt mask set/read register */
#define E1000E_IMC_OFFSET     0x000D8  /* interrupt mask clear */
#define E1000E_TIDV_OFFSET    0x03820  /* transmit interrupt delay value r/w */

// REGISTER BIT MASKS **********************************
#define E1000E_GCR_B22               (1<<22)
// Ctrl
#define E1000E_CTRL_FD               1           // full deplex
#define E1000E_CTRL_SLU              (1<<6)      // set link up
#define E1000E_CTRL_ILOS             (1<<7)      // loss of signal polarity
#define E1000E_CTRL_SPEED_10M        (0<<8)      // speed selection 10M
#define E1000E_CTRL_SPEED_100M       (1<<8)      // speed selection 100M
#define E1000E_CTRL_SPEED_1GV1       (2<<8)      // speed selection 1g v1
#define E1000E_CTRL_SPEED_1GV2       (3<<8)      // speed selection 1g v2
#define E1000E_CTRL_FRCSPD           (1<<11)     // force speed
#define E1000E_CTRL_FRCDPLX          (1<<12)     // force duplex
#define E1000E_CTRL_RST              (1<<26)     // reset
#define E1000E_CTRL_RFCE             (1<<27)     // receive flow control enable
#define E1000E_CTRL_TFCE             (1<<28)     // transmit control flow enable

// Status
#define E1000E_STATUS_FD             1           // full, half duplex = 1, 0
#define E1000E_STATUS_LU             (1<<1)      // link up established = 1
#define E1000E_STATUS_SPEED_10M      (0<<6)
#define E1000E_STATUS_SPEED_100M     (1<<6)
#define E1000E_STATUS_SPEED_1G       (2<<6)
#define E1000E_STATUS_SPEED_MASK     (3<<6)
#define E1000E_STATUS_ASDV_MASK      (3<<8)      // auto speed detect value
#define E1000E_STATUS_PHYRA          (1<<10)     // PHY required initialization

// E1000E Transmit Control Register
#define E1000E_TCTL_EN               (1 << 1)    // transmit enable
#define E1000E_TCTL_PSP              (1 << 3)    // pad short packet
#define E1000E_TCTL_CT               (0x0f << 4) // collision threshold
// collision distance full duplex 0x03f, 
#define E1000E_TCTL_COLD_FD          (0x03f << 12)
// collision distance half duplex 0x1ff 
#define E1000E_TCTL_COLD_HD          (0x1ff << 12)

// E1000E Transmit Descriptor Control
//// granularity thresholds unit 0b cache line, 1b descriptors
#define E1000E_TXDCTL_GRAN           (1<<24)
// CLEANUP
#define E1000E_TXDCTL_WTHRESH        0 //(1<<16)

// IPG = inter packet gap
#define E1000E_TIPG_IPGT             0x08          // IPG transmit time
#define E1000E_TIPG_IPGR1            (0x2 << 10)   // TIPG1 = 2
#define E1000E_TIPG_IPGR2            (0xa << 20)   // TIPG2 = 10

// E1000E Receive Control Register 
#define E1000E_RCTL_EN               (1 << 1)    // Receiver Enable
#define E1000E_RCTL_SBP              (1 << 2)    // Store Bad Packets
#define E1000E_RCTL_UPE              (1 << 3)    // Unicast Promiscuous Enabled
#define E1000E_RCTL_MPE              (1 << 4)    // Multicast Promiscuous Enabled
#define E1000E_RCTL_LPE              (1 << 5)    // Long Packet Reception Enable
#define E1000E_RCTL_LBM_NONE         (0 << 6)    // No Loopback
#define E1000E_RCTL_LBM_PHY          (3 << 6)    // PHY or external SerDesc loopback
#define E1000E_RCTL_RDMTS_HALF       (0 << 8)    // Free Buffer Threshold is 1/2 of RDLEN
#define E1000E_RCTL_RDMTS_QUARTER    (1 << 8)    // Free Buffer Threshold is 1/4 of RDLEN
#define E1000E_RCTL_RDMTS_EIGHTH     (2 << 8)    // Free Buffer Threshold is 1/8 of RDLEN
#define E1000E_RCTL_DTYP_LEGACY      (0 << 10)   // legacy descriptor type
#define E1000E_RCTL_DTYP_PACKET_SPLIT (1 << 10)   // packet split descriptor type
#define E1000E_RCTL_MO_36            (0 << 12)   // Multicast Offset - bits 47:36
#define E1000E_RCTL_MO_35            (1 << 12)   // Multicast Offset - bits 46:35
#define E1000E_RCTL_MO_34            (2 << 12)   // Multicast Offset - bits 45:34
#define E1000E_RCTL_MO_32            (3 << 12)   // Multicast Offset - bits 43:32
#define E1000E_RCTL_BAM              (1 << 15)   // Broadcast Accept Mode
#define E1000E_RCTL_VFE              (1 << 18)   // VLAN Filter Enable
#define E1000E_RCTL_CFIEN            (1 << 19)   // Canonical Form Indicator Enable
#define E1000E_RCTL_CFI              (1 << 20)   // Canonical Form Indicator Bit Value
#define E1000E_RCTL_DPF              (1 << 22)   // Discard Pause Frames
#define E1000E_RCTL_PMCF             (1 << 23)   // Pass MAC Control Frames
#define E1000E_RCTL_SECRC            (1 << 26)   // Strip Ethernet CRC

// Buffer Sizes
// RCTL.BSEX = 0
#define E1000E_RCTL_BSIZE_256        (3 << 16)
#define E1000E_RCTL_BSIZE_512        (2 << 16)
#define E1000E_RCTL_BSIZE_1024       (1 << 16)
#define E1000E_RCTL_BSIZE_2048       (0 << 16)
// RCTL.BSEX = 1
#define E1000E_RCTL_BSIZE_4096       ((3 << 16) | (1 << 25))
#define E1000E_RCTL_BSIZE_8192       ((2 << 16) | (1 << 25))
#define E1000E_RCTL_BSIZE_16384      ((1 << 16) | (1 << 25))
#define E1000E_RCTL_BSIZE_MASK       ~((3 << 16) | (1 << 25))

// RXDCTL
#define E1000E_RXDCTL_GRAN           (1 << 24)   // Granularity
#define E1000E_RXDCTL_WTHRESH        (1 << 16)   // number of written-back rxd
#define E1000E_RXDCTL_PTHRESH        (1 << 0)    // number of prefetching rxd
#define E1000E_RXDCTL_HTHRESH        (1 << 8)    // number of available host rxd

#define E1000E_RECV_BSIZE_256        256
#define E1000E_RECV_BSIZE_512        512
#define E1000E_RECV_BSIZE_1024       1024
#define E1000E_RECV_BSIZE_2048       2048
#define E1000E_RECV_BSIZE_4096       4096
#define E1000E_RECV_BSIZE_8192       8192
#define E1000E_RECV_BSIZE_16384      16384
#define E1000E_RECV_BSIZE_MAX        16384
#define E1000E_RECV_BSIZE_MIN        E1000E_RECV_BSIZE_256

// interrupt bits of icr register
#define E1000E_ICR_TXDW              1          // transmit descriptor written back 
#define E1000E_ICR_TXQE              (1 << 1)   // transmit queue empty 
#define E1000E_ICR_LSC               (1 << 2)   // link state change 
#define E1000E_ICR_RXO               (1 << 6)   // receive overrun 
#define E1000E_ICR_RXT0              (1 << 7)   // receiver timer interrupt 
#define E1000E_ICR_TXD_LOW           (1 << 15)  // transmit descriptor low threshold hit 
#define E1000E_ICR_SRPD              (1 << 16)  // small receive packet detected
#define E1000E_ICR_RXQ0              (1 << 20)  // receive queue 0 interrupt
#define E1000E_ICR_RXQ1              (1 << 21)  // receive queue 1 interrupt
#define E1000E_ICR_TXQ0              (1 << 22)  // transmit queue 0 interrupt
#define E1000E_ICR_TXQ1              (1 << 23)  // transmit queue 1 interrupt
#define E1000E_ICR_OTHER             (1 << 24)  // other interrupts
#define E1000E_ICR_INT_ASSERTED      (1 << 31)  // interrupt asserted

#define E1000E_RDTR_FPD              (1 << 31)  // flush partial descriptor block

// new type declaration
struct e1000e_dev {
  // for our linked list of virtio devices
  struct nk_net_dev *netdev;
  // a pointer to the base class
  struct list_head e1000e_node;
  struct pci_dev *pci_dev;
  // pci interrupt and interupt vector
  uint8_t   pci_intr;  // number on bus
  uint8_t   intr_vec;  // number we will see
  // Where registers are mapped into the I/O address space
  uint16_t  ioport_start;
  uint16_t  ioport_end;  
  // Where registers are mapped into the physical memory address space
  uint64_t  mem_start;
  uint64_t  mem_end;
};

struct e1000e_desc_ring {
  void *ring_buffer;
  uint32_t head_prev;
  uint32_t tail_pos;
  uint32_t count;
  void *packet_buffer;
  uint32_t blocksize;
};

struct e1000e_rx_desc {
  uint64_t* addr;
  uint16_t length;
  uint16_t checksum;
  struct {
    uint8_t dd    : 1;
    uint8_t eop   : 1;
    uint8_t ixsm  : 1;
    uint8_t vp    : 1;
    uint8_t rsv   : 1;
    uint8_t tcpcs : 1;
    uint8_t ipcs  : 1;
    uint8_t pif   : 1;
  } status;
  uint8_t errors;
  uint16_t special;
} __attribute__((packed));

// legacy mode
struct e1000e_tx_desc {
  uint64_t* addr;
  uint16_t length;
  uint8_t  cso;
  struct {
    uint8_t eop : 1;
    uint8_t ifcs: 1;
    uint8_t ic  : 1;
    uint8_t rs  : 1;
    uint8_t rsvd: 1;
    uint8_t dext: 1;
    uint8_t vle : 1;
    uint8_t ide : 1;
  } cmd;
  struct {
    uint8_t dd    : 1;
    uint8_t ec    : 1;
    uint8_t lc    : 1;
    uint8_t rsvtu : 1;
    uint8_t rsvd2 : 4;
  } status;
  uint8_t css;
  uint16_t special;
} __attribute__((packed)); 

// TODO change the type of context to void
struct e1000e_fn_map {
  void (*callback)(nk_net_dev_status_t, void *);
  uint64_t *context;
};

struct e1000e_map_ring {
  struct e1000e_fn_map *map_ring;
  // head and tail positions of the fn_map ring queue
  uint64_t head_pos;
  uint64_t tail_pos;
  // the number of elements in the map ring buffer
  uint64_t ring_len;
};
  
struct e1000e_state {
  // FIX compared with the e1000_pci.c
  // delete dev
  // add all the fields in dev to this structure
  char name[DEV_NAME_LEN];
  uint8_t mac_addr[6];
  // the bus number of the devince on the pci bus
  uint32_t bus_num;       
  // the device number of the device on the pci bus
  uint32_t dev_num;
  struct e1000e_dev *dev;
  struct e1000e_desc_ring *tx_ring;
  struct e1000e_desc_ring *rxd_ring;
  // a circular queue mapping between callback function and tx descriptor
  struct e1000e_map_ring *tx_map;
  // a circular queue mapping between callback funtion and rx descriptor
  struct e1000e_map_ring *rx_map;
  uint64_t rx_buffer_size;
};

// function declaration
int e1000e_post_send(void*, uint8_t*, uint64_t,
                    void (*)(nk_net_dev_status_t, void*), void*);
int e1000e_post_receive(void*, uint8_t*, uint64_t,
                       void (*)(nk_net_dev_status_t, void*), void*);
void e1000e_disable_all_int();
void e1000e_trigger_int();
void e1000e_trigger_int_num(uint32_t int_num);
void e1000e_legacy_int_off();
void e1000e_legacy_int_on();
int e1000e_pci_init(struct naut_info * naut);
int e1000e_pci_deinit();
void e1000e_interpret_int_shell();
void e1000e_interpret_int(uint32_t);
void e1000e_interpret_ims(struct e1000e_state*);
void e1000e_interpret_icr(struct e1000e_state*);
void e1000e_read_stat_shell();
void e1000e_reset_all_int();
void e1000e_reset_rxo();
void e1000e_interpret_rxd(struct e1000e_state*);
void e1000e_interpret_rxd_shell();
#endif
