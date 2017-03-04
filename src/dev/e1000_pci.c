#include <nautilus/nautilus.h>
#include <nautilus/netdev.h>
#include <nautilus/cpu.h> // warrior
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
/*
#define IO_READ(d, o) inl((volatile uint32_t*)(((d)->ioport_start)+(o)))
#define IO_WRITE(d, o, v) outl((v),(volatile uint32_t*)(((d)->ioport_start)+(o)))
#define IODATA_READ(d) (IO_READ((d), 0x4))
#define IODATA_WRITE(d, v) (IO_WRITE((d), 0x4, (v)))
#define IOADDR_WRITE(d, v) (IO_WRITE((d), 0x0, (v)))*/
// warrior -> try this way later V
#define IO_READ(d, o) (*((volatile uint32_t*)(((d)->ioport_start)+(o))))
#define IO_WRITE(d, o, v) ((*((volatile uint32_t*)(((d)->ioport_start)+(o))))=(v))
#define IODATA_READ(d) (IO_READ((d), (volatile uint32_t*)0x4))
#define IODATA_WRITE(d, v) (IO_WRITE((d), (volatile uint32_t*)0x4, (v)))
#define IOADDR_WRITE(d, v) (IO_WRITE((d), (volatile uint32_t*)0x0, (v)))

// linked list of e1000 devices
// static global var to this only file
static struct list_head dev_list;
// TODO allocate rx_desc and tx_desc for only one packet -> 4 descriptors
static struct e1000_rx_desc rx_desc[4];
static struct e1000_tx_desc tx_desc[4];
// transmitting buffer
static void *td_buffer;
static struct e1000_dev *vdev;

/*
static struct nk_net_dev_int {
    get_characteristic = e1000_get_charactestic,
    post_receive = e1000_post_receive,
    post_send = e1000_post_send,
	} e1000_inter;
*/

uint8_t my_packet[1000] = {
    0xff,
    0xff,
    0xff,
    0xff,
    0xff,
    0xff,

    0x01,
    0x02,
    0x03,
    0x04,
    0x05,
    0x06,

    0x80,
    0x00,
    
    0xde,
    0xad,
    0xbe,
    0xef,
};

static int e1000_send_packet(void* packet_addr, uint16_t packet_size){
  uint64_t tdt = READ(vdev, TDT_OFFSET); // get current tail
  struct e1000_tx_desc *d  = (struct e1000_tx_desc *)((char*)td_buffer + sizeof(struct e1000_tx_desc) * tdt);

  memset(d,0,sizeof(struct e1000_tx_desc));

  DEBUG("td buffer=%d\n", td_buffer); 
  DEBUG("sizeof descriptor=%d\n", sizeof(struct e1000_tx_desc)); 
  DEBUG("descriptor addr=%d\n", d); 
  DEBUG("TDT=%d\n", tdt);  

  d->cmd.dext = 0;
  d->cmd.vle = 0;
  d->cmd.eop = 1;
  d->cmd.ifcs = 1;  
  d->cmd.rs = 1;  
  d->addr = packet_addr;
  d->length = packet_size;
  
  DEBUG("Sizeof e1000_tx_desc is %d\n",sizeof(struct e1000_tx_desc));

  WRITE(vdev, TDT_OFFSET, tdt+1); //increment transmit descriptor list tail by 1
  DEBUG("TDT=%d\n", READ(vdev, TDT_OFFSET));  
  DEBUG("TDH=%d\n", READ(vdev, TDH_OFFSET));
  DEBUG("e1000 status=0x%x\n", READ(vdev, 0x8));  
  return 0;
}



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
      if (cfg->vendor_id==VENDOR_ID && cfg->device_id==DEVICE_ID) {
        DEBUG("E1000 Device Found\n");
        // struct e1000_dev *vdev;
  
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
        DEBUG("total pkt tx=%d\n", READ(vdev, TPT_OFFSET));
        DEBUG("total pkt rx=%d\n", READ(vdev, TPR_OFFSET));
        /*
          nk_net_dev_register("e1000-1",
			    0,
			    &e1000_inter, 
			    state); // the e1000 structure
        */

        uint16_t old_cmd = pci_cfg_readw(bus->num,pdev->num,0,0x4);
	
        DEBUG("Old PCI CMD: %x\n",old_cmd);

        old_cmd |= 0x7;  // make sure bus master is enabled
        old_cmd &= ~0x40;

        DEBUG("New PCI CMD: %x\n",old_cmd);

        pci_cfg_writew(bus->num,pdev->num,0,0x4,old_cmd);

        uint16_t stat = pci_cfg_readw(bus->num,pdev->num,0,0x6);

        DEBUG("PCI STATUS: %x\n",stat);

        // read the status register at void ptr + offset
        // uint32_t status=*(volatile uint32_t *)(vdev->mem_start+0x8);
        uint32_t status=READ(vdev, 0x8);
        DEBUG("e1000 status=0x%x\n", status);
        uint32_t mac_low=READ(vdev, RAL_OFFSET);
        uint32_t mac_high=READ(vdev, RAH_OFFSET);        
        uint64_t macall=((uint64_t)mac_low+((uint64_t)mac_high<<32))&(0xffffffffffffffff >> 12);
        DEBUG("e1000 mac=0x%lX\n", macall);        
        DEBUG("e1000 low_mac=0x%X\n", mac_low);        
        list_add(&dev_list, &vdev->e1000_node);
      }      
    }
  }

  // allocate transmitting buffer for 64kB.
  // td_buffer = memalign(16, 64*1024);
  // FIXME 16byte aligned 64kB, min = 128b
  td_buffer = malloc(64*1024);
  if (!td_buffer) {
    ERROR("Cannot allocate tx buffer\n");
    return -1;
  }

  DEBUG("TX BUFFER AT %p\n",td_buffer); // we need this to be < 4GB

  // store the address of the memory in TDBAL/TDBAH
  WRITE(vdev, TDBAL_OFFSET, (uint32_t)(0x00000000ffffffff & (uint64_t) td_buffer));
  WRITE(vdev, TDBAH_OFFSET, (uint32_t)((0xffffffff00000000 & (uint64_t) td_buffer) >> 32));
  DEBUG("td_buffer=0x%016lx, TDBAL=0x%08x, TDBAH=0x%08x\n",
        td_buffer, READ(vdev, TDBAL_OFFSET), READ(vdev, TDBAH_OFFSET));
  // write tdlen
  WRITE(vdev, TDLEN_OFFSET, 64*1024);
  // write the tdh, tdt with 0
  WRITE(vdev, TDT_OFFSET, 0);
  WRITE(vdev, TDH_OFFSET, 0);
  // write tctl register
  WRITE(vdev, TCTL_OFFSET, 0x4010a);
  // write tipg regsiter     00,00 0000 0110,0000 0010 00,00 0000 1010
  WRITE(vdev, TIPG_OFFSET, 0x0060200a); // will be zero when emulating hardware
  DEBUG("TDLEN=0x%08x, TDH=0x%08x, TDT=0x%08x, TCTL=0x%08x, TIPG=0x%08x\n",
        READ(vdev, TDLEN_OFFSET), READ(vdev, TDH_OFFSET), READ(vdev, TDT_OFFSET),
        READ(vdev, TCTL_OFFSET), READ(vdev, TIPG_OFFSET));

  // ***************** INIT IS COMPLETE ************************* //
  
  e1000_send_packet(my_packet, (uint16_t)sizeof(my_packet));
  e1000_send_packet(my_packet, (uint16_t)sizeof(my_packet));
  DEBUG("total pkt tx=%d\n", READ(vdev, TPT_OFFSET));
  DEBUG("total pkt tx=%d\n", READ(vdev, TPT_OFFSET));
  return 0;
}

int e1000_pci_deinit()
{
  INFO("deinited\n");
  return 0;
}

