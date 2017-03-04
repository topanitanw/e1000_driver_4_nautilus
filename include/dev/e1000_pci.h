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

// new type declaration
struct e1000_dev {
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
  volatile uint8_t status:4;
  volatile uint8_t rsvd2:4;
  volatile uint8_t css;
  volatile uint16_t special;
} __attribute__((packed)); 

// function declaration
int e1000_pci_init(struct naut_info * naut);
int e1000_pci_deinit();

#endif
