#ifndef __E1000_PCI
#define __E1000_PCI

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
  volatile uint8_t status;
  volatile uint8_t errors;
  volatile uint16_t special;
} __attribute__((packed));

struct e1000_tx_desc {
  volatile uint64_t addr;
  volatile uint16_t length;
  volatile uint8_t cso;
  volatile uint8_t cmd;
  volatile uint8_t status;
  volatile uint8_t css;
  volatile uint16_t special;
} __attribute__((packed));

// function declaration
int e1000_pci_init(struct naut_info * naut);
int e1000_pci_deinit();

#endif
