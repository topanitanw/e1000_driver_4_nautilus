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
 * Copyright (c) 2017, Panitan Wongse-ammat, Marc Warrior, Galen Lansbury 
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

#ifndef __E1000_PCI
#define __E1000_PCI

// Constant variables
// These variables are configurable, and they will change the number of descriptors
// and the packet buffer pointed by the descriptors.
// The number of descriptors is always a multiple of eight.

#define NIC_NAME              "e1000e-0"
#define TX_DSC_COUNT          16
#define TX_BLOCKSIZE          256 // bytes available per DMA block
#define RX_DSC_COUNT          16  // equal to DMA block count
#define RX_BLOCKSIZE          256 // bytes available per DMA block

// After this line, PLEASE DO NOT CHANGE ANYTHING.
// transmission unit
// Data sheet from page 36
// 16384 bytes is from the maximum packet buffer size that e1000 can receive.
// 16288 bytes is the maximum packet size that e1000 can send in theory.
// Ethernet standard MTU is 1500 bytes.
#define MAX_TU                16384    /* maximum transmission unit */
#define MIN_TU                48       /* minimum transmission unit */

// PCI CONFIG SPACE ************************************
#define INTEL_VENDOR_ID       0x8086
#define E1000_DEVICE_ID       0x100E
#define E1000_CTRL_OFFSET     0x00000  /* Device Control - RW */
#define E1000_STATUS_OFFSET   0x00008  /* Device Status - RO */

// REGISTER OFFSETS ************************************
#define TDBAL_OFFSET          0x3800   // transmit descriptor base address low (64b)
#define TDBAH_OFFSET          0x3804   // transmit descriptor base address high
#define TDLEN_OFFSET          0x3808   // transmit descriptor list length
#define TDH_OFFSET            0x3810   // transmit descriptor head
#define TDT_OFFSET            0x3818   // transmit descriptor tail
#define TCTL_OFFSET           0x0400   // transmit control
#define TIPG_OFFSET           0x0410   // transmit interpacket gap
#define E1000_TXDCTL_OFFSET   0x03828  // transmit descriptor control r/w
 
#define E1000_RAL_OFFSET      0x5400   // receive address (64b)
#define E1000_RAH_OFFSET      0x5404   //  

#define RDBAL_OFFSET          0x2800   // receive descriptor base address low
#define RDBAH_OFFSET          0x2804   // receive descriptor base address high
#define RDLEN_OFFSET          0x2808   // receive descriptor list length
#define RDH_OFFSET            0x2810   // receive descriptor head
#define RDT_OFFSET            0x2818   // receive descriptor tail
#define RCTL_OFFSET           0x0100   // receive control
#define E1000_RDTR_OFFSET     0x2820   // receive delay timer
#define E1000_RSRPD_OFFSET    0x02C00  // receive small packet detect interrupt r/w

#define E1000_TPT_OFFSET      0x40D4   // total package transmit
#define E1000_TPR_OFFSET      0x40D0   // total package receive
#define E1000_COLC_OFFSET     0x04028  // collision count
#define E1000_RXERRC_OFFSET   0x0400C  // rx error count

// 4 interrupt register offset
#define E1000_ICR_OFFSET      0x000C0  /* interrupt cause read register */
#define E1000_ICS_OFFSET      0x000C8  /* interrupt cause set register */
#define E1000_IMS_OFFSET      0x000D0  /* interrupt mask set/read register */
#define E1000_IMC_OFFSET      0x000D8  /* interrupt mask clear */
#define E1000_TIDV_OFFSET     0x03820  /* transmit interrupt delay value r/w */

// REGISTER BIT MASKS **********************************
// E1000 Transmit Control Register
#define E1000_TCTL_EN               (1 << 1)      // transmit enable
#define E1000_TCTL_PSP              (1 << 3)      // pad short packet
#define E1000_TCTL_CT               (0x0f << 4)   // collision threshold
#define E1000_TCTL_COLD_FD          (0x40 << 12)  // collision distance full duplex
#define E1000_TCTL_COLD_HD          (0x200 << 12) // collision distance half duplex
// IPG = inter packet gap
#define E1000_TIPG_IPGT             0x0a          // IPG transmit time
#define E1000_TIPG_IPGR1_IEEE8023   (0x8 << 10)   // TIPG1 for IEEE802.3
#define E1000_TIPG_IPGR2_IEEE8023   (0x6 << 20)   // TIPG2 for IEEE802.3

// E1000 Receive Control Register 
#define E1000_RCTL_EN               (1 << 1)      // Receiver Enable
#define E1000_RCTL_SBP              (1 << 2)      // Store Bad Packets
#define E1000_RCTL_UPE              (1 << 3)      // Unicast Promiscuous Enabled
#define E1000_RCTL_MPE              (1 << 4)      // Multicast Promiscuous Enabled
#define E1000_RCTL_LPE              (1 << 5)      // Long Packet Reception Enable
#define E1000_RCTL_LBM_NONE         (0 << 6)      // No Loopback
#define E1000_RCTL_LBM_PHY          (3 << 6)      // PHY or external SerDesc loopback
#define E1000_RCTL_RDMTS_HALF       (0 << 8)      // Free Buffer Threshold is 1/2 of RDLEN
#define E1000_RCTL_RDMTS_QUARTER    (1 << 8)      // Free Buffer Threshold is 1/4 of RDLEN
#define E1000_RCTL_RDMTS_EIGHTH     (2 << 8)    // Free Buffer Threshold is 1/8 of RDLEN
#define E1000_RCTL_MO_36            (0 << 12)   // Multicast Offset - bits 47:36
#define E1000_RCTL_MO_35            (1 << 12)   // Multicast Offset - bits 46:35
#define E1000_RCTL_MO_34            (2 << 12)   // Multicast Offset - bits 45:34
#define E1000_RCTL_MO_32            (3 << 12)   // Multicast Offset - bits 43:32
#define E1000_RCTL_BAM              (1 << 15)   // Broadcast Accept Mode
#define E1000_RCTL_VFE              (1 << 18)   // VLAN Filter Enable
#define E1000_RCTL_CFIEN            (1 << 19)   // Canonical Form Indicator Enable
#define E1000_RCTL_CFI              (1 << 20)   // Canonical Form Indicator Bit Value
#define E1000_RCTL_DPF              (1 << 22)   // Discard Pause Frames
#define E1000_RCTL_PMCF             (1 << 23)   // Pass MAC Control Frames
#define E1000_RCTL_SECRC            (1 << 26)   // Strip Ethernet CRC
 
// Buffer Sizes
// RCTL.BSEX = 0
#define E1000_RCTL_BSIZE_256        (3 << 16)
#define E1000_RCTL_BSIZE_512        (2 << 16)
#define E1000_RCTL_BSIZE_1024       (1 << 16)
#define E1000_RCTL_BSIZE_2048       (0 << 16)
// RCTL.BSEX = 1
#define E1000_RCTL_BSIZE_4096       ((3 << 16) | (1 << 25))
#define E1000_RCTL_BSIZE_8192       ((2 << 16) | (1 << 25))
#define E1000_RCTL_BSIZE_16384      ((1 << 16) | (1 << 25))
#define E1000_RCTL_BSIZE_MASK       ~((3 << 16) | (1 << 25))

#define E1000_RECV_BSIZE_256        256
#define E1000_RECV_BSIZE_512        512
#define E1000_RECV_BSIZE_1024       1024
#define E1000_RECV_BSIZE_2048       2048
#define E1000_RECV_BSIZE_4096       4096
#define E1000_RECV_BSIZE_8192       8192
#define E1000_RECV_BSIZE_16384      16384
#define E1000_RECV_BSIZE_MAX        16384

// interrupt bits of icr register
#define E1000_ICR_TXDW              1          // transmit descriptor written back 
#define E1000_ICR_TXQE              (1 << 1)   // transmit queue empty 
#define E1000_ICR_TXD_LOW           (1 << 15)  // transmit descriptor low threshold hit 

#define E1000_ICR_SRPD              (1 << 16)  // small receive packet detected 
#define E1000_ICR_RXT0              (1 << 7)   // receiver timer interrupt 
#define E1000_ICR_RXO               (1 << 6)   // receive overrun 
#define E1000_ICR_LSC               (1 << 2)   // link state change 

// new type declaration
struct e1000_dev {
  // for our linked list of virtio devices
  struct nk_net_dev *netdev;
  // a pointer to the base class
  struct list_head e1000_node;
  // pci interrupt and interupt vector
  struct pci_dev *pci_dev;
  uint8_t   pci_intr;  // number on bus
  uint8_t   intr_vec;  // number we will see
  // Where registers are mapped into the I/O address space
  uint16_t  ioport_start;
  uint16_t  ioport_end;  
  // Where registers are mapped into the physical memory address space
  uint64_t  mem_start;
  uint64_t  mem_end;
};

struct e1000_desc_ring {
  void *ring_buffer;
  uint32_t head_prev;
  uint32_t tail_pos;
  uint32_t count;
  void *packet_buffer;
  uint32_t blocksize;
};

struct e1000_rx_desc {
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
struct e1000_tx_desc {
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

struct e1000_fn_map {
  void (*callback)(nk_net_dev_status_t, void *);
  uint64_t *context;
};

struct e1000_map_ring {
  struct e1000_fn_map *map_ring;
  // head and tail positions of the fn_map ring queue
  uint64_t head_pos;
  uint64_t tail_pos;
  // the number of elements in the map ring buffer
  uint64_t ring_len;
};
  
struct e1000_state {
  char name[DEV_NAME_LEN];
  uint8_t mac_addr[6];
  struct e1000_dev *dev;
  struct e1000_desc_ring *tx_ring;
  struct e1000_desc_ring *rxd_ring;
  // a circular queue mapping between callback function and tx descriptor
  struct e1000_map_ring *tx_map;
  // a circular queue mapping between callback funtion and rx descriptor
  struct e1000_map_ring *rx_map;
  uint64_t rx_buffer_size;
};

// function declaration
void nk_ifconfig();
int e1000_post_send(void*, uint8_t*, uint64_t,
                    void (*)(nk_net_dev_status_t, void*), void*);
int e1000_post_receive(void*, uint8_t*, uint64_t,
                       void (*)(nk_net_dev_status_t, void*), void*);
int e1000_pci_init(struct naut_info * naut);
int e1000_pci_deinit();

#endif
