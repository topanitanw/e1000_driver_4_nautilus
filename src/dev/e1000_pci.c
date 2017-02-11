#include <nautilus/nautilus.h>
#include <dev/pci.h>
#include <dev/e1000_pci.h>

#ifndef NAUT_CONFIG_DEBUG_E1000_PCI
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...)
#endif 

#define INFO(fmt, args...) INFO_PRINT("E1000_PCI: " fmt, ##args)
#define DEBUG(fmt, args...) DEBUG_PRINT("E1000_PCI: DEBUG: " fmt, ##args)
#define ERROR(fmt, args...) ERROR_PRINT("E1000_PCI: ERROR: " fmt, ##args)
#define READ(d, o) (*((volatile uint32_t*)(((d)->mem_start)+(o))))
#define WRITE(d, o, v) ((*((volatile uint32_t*)(((d)->mem_start)+(o))))=(v))
#define READL(d, o) (*((volatile uint64_t*)(((d)->mem_start)+(o))))
#define WRITEL(d, o, v) ((*((volatile uint64_t*)(((d)->mem_start)+(o))))=(v))

// linked list of e1000 devices
// static global var to this only file
static struct list_head dev_list;

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

int e1000_pci_init(struct naut_info * naut)
{
  struct pci_info *pci = naut->sys.pci;
  struct list_head *curbus, *curdev;

  INFO("init\n");
  // building the e1000 linked list and register it
  INIT_LIST_HEAD(&dev_list);

  if (!pci) { 
    ERROR("No PCI info\n");
    return -1;
  }

  DEBUG("Finding e1000 devices\n");

  list_for_each(curbus,&(pci->bus_list)) { 
    struct pci_bus *bus = list_entry(curbus,struct pci_bus,bus_node);

    DEBUG("Searching PCI bus %u for E1000 devices\n", bus->num);

    list_for_each(curdev, &(bus->dev_list)) { 
      struct pci_dev *pdev = list_entry(curdev,struct pci_dev,dev_node);
      struct pci_cfg_space *cfg = &pdev->cfg;

      DEBUG("Device %u is a %x:%x\n", pdev->num, cfg->vendor_id, cfg->device_id);
      // intel vendor id and e1000 device id
      if (cfg->vendor_id==0x8086 && cfg->device_id==0x100E) {
        DEBUG("E1000 Device Found\n");
        struct e1000_dev *vdev;
  
        vdev = malloc(sizeof(struct e1000_dev));
        if (!vdev) {
          ERROR("Cannot allocate device\n");
          return -1;
        }

        memset(vdev,0,sizeof(*vdev));
	
        vdev->pci_dev = pdev;

        // PCI Interrupt (A..D)
        vdev->pci_intr = cfg->dev_cfg.intr_pin;
        // Figure out mapping here or look at capabilities for MSI-X
        // vdev->intr_vec = ...

        // TODO(Panitan) find out the bar for e1000
        for (int i=0;i<6;i++) { 
          uint32_t bar = pci_cfg_readl(bus->num,pdev->num, 0, 0x10 + i*4);
          uint32_t size;
          DEBUG("bar %d: 0x%0x\n",i, bar);
          // go through until the last one, and get out of the loop
          if (bar==0) { 
            break;
          }
          // get the last bit and if it is zero, it is the memory
          // " -------------------------"  one, it is the io
          if (!(bar & 0x1)) { 
            uint8_t mem_bar_type = (bar & 0x6) >> 1;
            if (mem_bar_type != 0) { // 64 bit address that we do not handle it
              ERROR("Cannot handle memory bar type 0x%x\n", mem_bar_type);
              return -1;
            }
          }

          // determine size
          // write all 1s, get back the size mask
          pci_cfg_writel(bus->num,pdev->num,0,0x10 + i*4, 0xffffffff);
          // size mask comes back + info bits
          // write all ones and read back. if we get 00 (negative size), size = 4.
          size = pci_cfg_readl(bus->num,pdev->num,0,0x10 + i*4);

          // mask all but size mask
          if (bar & 0x1) { 
            // I/O
            size &= 0xfffffffc;
          } else {
            // memory
            size &= 0xfffffff0;
          }
          // two complement, get back the positive size
          size = ~size;
          size++; 

          // now we have to put back the original bar
          pci_cfg_writel(bus->num,pdev->num,0,0x10 + i*4, bar);

          if (!size) { 
            // non-existent bar, skip to next one
            continue;
          }
          uint32_t start;
          if (bar & 0x1) { 
            start = vdev->ioport_start = bar & 0xffffffc0;
            vdev->ioport_end = vdev->ioport_start + size;
          } else {
            start = vdev->mem_start = bar & 0xfffffff0;
            vdev->mem_end = vdev->mem_start + size;
          }
          DEBUG("bar %d is %s address=0x%x size=0x%x\n", i,
                bar & 0x1 ? "io port":"memory",
                start, size);
        }

        INFO("Adding e1000 device: bus=%u dev=%u func=%u: pci_intr=%u intr_vec=%u ioport_start=%p ioport_end=%p mem_start=%p mem_end=%p\n",
             bus->num, pdev->num, 0,
             vdev->pci_intr, vdev->intr_vec,
             vdev->ioport_start, vdev->ioport_end,
             vdev->mem_start, vdev->mem_end);
        // read the status register at void ptr + offset
        // uint32_t status=*(volatile uint32_t *)(vdev->mem_start+0x8);
        uint32_t status=READ(vdev, 0x8);
        DEBUG("e1000 status=0x%x\n", status);
        uint32_t mac_low=READ(vdev, 0x5400);
        uint32_t mac_high=READ(vdev, 0x5404);        
        uint64_t macall=((uint64_t)mac_low+((uint64_t)mac_high<<32))&(0xffffffffffffffff >> 12);
        DEBUG("e1000 mac=0x%lX\n", macall);        
        DEBUG("e1000 low_mac=0x%X\n", mac_low);        
        list_add(&dev_list, &vdev->e1000_node);
      }
      
    }
  }
  return 0;
}

int e1000_pci_deinit()
{
  INFO("deinited\n");
  return 0;
}
