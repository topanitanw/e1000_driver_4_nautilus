#ifndef __E1000_PCI
#define __E1000_PCI

// REGISTER OFFSETS ************************************
#define RAL_OFFSET      0x5400 // receive address (64b)
#define RAH_OFFSET      0x5404 //  

#define RDBAL_OFFSET    0x2800 // receive descriptor list address (64b)
#define RDBAH_OFFSET    0x2804
#define RDLEN_OFFSET    0x2808 // receive descriptor list length
#define RDH_OFFSET      0x2810 // receive descriptor head
#define RDT_OFFSET      0x2818 // receive descriptor tail
#define RCTL_OFFSET     0x0100 // receive control
#define RDTR_OFFSET     0x2820 // receive delay timer

#define TPT_OFFSET      0x40D4 // total package transmit
#define TPR_OFFSET      0x40D0 // total packagte receive
#define TDBAL_OFFSET    0x3800 // transmit descriptor list address (64b)
#define TDBAH_OFFSET    0x3804 
#define TDLEN_OFFSET    0x3808 // transmit descriptor list length
#define TDH_OFFSET      0x3810 // transmit descriptor head
#define TDT_OFFSET      0x3818 // transmit descriptor tail
#define TCTL_OFFSET     0x0400 // transmit control
#define TIPG_OFFSET     0x0410 // transmit interpacket gap

// PCI CONFIG SPACE ************************************
#define VENDOR_ID 0x8086
#define DEVICE_ID 0x100E


// REGISTER BIT MASKS **********************************
// E1000 Receive Register 
#define RCTL_EN                         (1 << 1)    // Receiver Enable
#define RCTL_SBP                        (1 << 2)    // Store Bad Packets
#define RCTL_UPE                        (1 << 3)    // Unicast Promiscuous Enabled
#define RCTL_MPE                        (1 << 4)    // Multicast Promiscuous Enabled
#define RCTL_LPE                        (1 << 5)    // Long Packet Reception Enable
#define RCTL_LBM_NONE                   (0 << 6)    // No Loopback
#define RCTL_LBM_PHY                    (3 << 6)    // PHY or external SerDesc loopback
#define RTCL_RDMTS_HALF                 (0 << 8)    // Free Buffer Threshold is 1/2 of RDLEN
#define RTCL_RDMTS_QUARTER              (1 << 8)    // Free Buffer Threshold is 1/4 of RDLEN
#define RTCL_RDMTS_EIGHTH               (2 << 8)    // Free Buffer Threshold is 1/8 of RDLEN
#define RCTL_MO_36                      (0 << 12)   // Multicast Offset - bits 47:36
#define RCTL_MO_35                      (1 << 12)   // Multicast Offset - bits 46:35
#define RCTL_MO_34                      (2 << 12)   // Multicast Offset - bits 45:34
#define RCTL_MO_32                      (3 << 12)   // Multicast Offset - bits 43:32
#define RCTL_BAM                        (1 << 15)   // Broadcast Accept Mode
#define RCTL_VFE                        (1 << 18)   // VLAN Filter Enable
#define RCTL_CFIEN                      (1 << 19)   // Canonical Form Indicator Enable
#define RCTL_CFI                        (1 << 20)   // Canonical Form Indicator Bit Value
#define RCTL_DPF                        (1 << 22)   // Discard Pause Frames
#define RCTL_PMCF                       (1 << 23)   // Pass MAC Control Frames
#define RCTL_SECRC                      (1 << 26)   // Strip Ethernet CRC
 
// Buffer Sizes
#define RCTL_BSIZE_256                  (3 << 16)
#define RCTL_BSIZE_512                  (2 << 16)
#define RCTL_BSIZE_1024                 (1 << 16)
#define RCTL_BSIZE_2048                 (0 << 16)
#define RCTL_BSIZE_4096                 ((3 << 16) | (1 << 25))
#define RCTL_BSIZE_8192                 ((2 << 16) | (1 << 25))
#define RCTL_BSIZE_16384                ((1 << 16) | (1 << 25))

// VARIABLE CONSTANTS *************************************
/* these may need to dynamically change for different machines
   (for example, to allow buffer sizes to reflect mem availabilty)
*/
#define TX_DSC_COUNT 128
#define TX_BLOCKSIZE 256 // bytes available per DMA block
#define RX_DSC_COUNT 64 // equal to DMA block count
#define RX_BLOCKSIZE 256 // bytes available per DMA block

// new type declaration
struct e1000_dev {
  struct nk_net_dev *netdev;
  // for our linked list of virtio devices
  struct list_head e1000_node;
  // a pointer to the base class
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

struct e1000_ring {
  volatile void     *ring_buffer;
  volatile uint8_t  head_prev;
  volatile uint8_t  tail_pos;
  int count;
  void *packet_buffer;
  int blocksize;
};

struct e1000_rx_desc {
  volatile uint64_t addr;
  volatile uint16_t length;
  volatile uint16_t checksum;
  volatile struct {
      uint8_t dd    : 1;
      uint8_t eop   : 1;
      uint8_t ixsm  : 1;
      uint8_t vp    : 1;
      uint8_t rsv   : 1;
      uint8_t tcpcs : 1;
      uint8_t ipcs  : 1;
      uint8_t pif   : 1;
  } status;
  volatile uint8_t errors;
  volatile uint16_t special;
} __attribute__((packed));

// legacy mode
struct e1000_tx_desc {
  volatile uint64_t addr;
  volatile uint16_t length;
  volatile uint8_t  cso;
  volatile struct {
    uint8_t eop : 1;
    uint8_t ifcs: 1;
    uint8_t ic  : 1;
    uint8_t rs  : 1;
    uint8_t rsvd: 1;
    uint8_t dext: 1;
    uint8_t vle : 1;
    uint8_t ide : 1;
  } cmd;
  volatile struct {
    uint8_t dd    : 1;
    uint8_t ec    : 1;
    uint8_t lc    : 1;
    uint8_t rsvtu : 1;
    uint8_t rsvd2 : 4;
  } status;
  volatile uint8_t css;
  volatile uint16_t special;
} __attribute__((packed)); 

struct e1000_state {
    volatile struct e1000_ring *rx_ring;
    volatile struct e1000_ring *tx_ring;
    char name[DEV_NAME_LEN];
    struct e1000_dev *dev;
    uint64_t mac_addr;
};

// function declaration
int e1000_pci_init(struct naut_info * naut);
int e1000_pci_deinit();

#endif
