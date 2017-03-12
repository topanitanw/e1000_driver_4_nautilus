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
#define RXD_STATUS(i)  (((struct e1000_rx_desc*)rx_desc_ring->ring_buffer)[(i)].status)
#define RXD_LENGTH(i)  (((struct e1000_rx_desc*)rx_desc_ring->ring_buffer)[(i)].length)
#define RXD_ADDR(i)  ((uint64_t*)(((struct e1000_rx_desc*)rx_desc_ring->ring_buffer)[(i)].addr))

// linked list of e1000 devices
// static global var to this only file
static struct list_head dev_list;
// transmitting buffer
static void *td_buffer;
static int td_buff_size;
static void *rd_buffer;
static void *rcv_buffer;
static int rcv_desc_count;
static int rcv_block_size; // TODO make this constant
static int rd_buff_size;
static struct e1000_dev *vdev;
static struct e1000_rx_ring *rx_desc_ring;

int my_packet[16] = {0xdeadbeef,0xbeefdead,};
    
/*
Description of Receive process

1) recieve descriptor must fit inside of the ring of 1024 (arbritary)
	however, all of these recieve have headers are headers that point to blocks of memory
				(where the device is going to automatically store the incoming data from packets)
				The number of recieve descriptors is set by the size (16 receive descriptors can fit in 1k)
		The ring is an array of receive descriptors (the address part consumes 8 bytes of each recieve descriptor)
			(other 8 bytes are about the status and size, etc..., [and device will change this])	
	The receive buffer is an array of blocks that the recieve descriptors are pointing too
*/	

static int e1000_init_single_rxd(int index){
  // initialize single descriptor pointing to where device can write rcv'd packet
    struct e1000_rx_desc tmp_rxd;
    memset(&tmp_rxd, 0, sizeof(struct e1000_rx_desc));
    tmp_rxd.addr = (uint64_t)((uint8_t*)rcv_buffer + rcv_block_size*index);
    ((struct e1000_rx_desc *)rx_desc_ring->ring_buffer)[index] = tmp_rxd;
    return 0;
}

// initialize a ring buffer to hold receive descriptor and 
// another buffer for DMA space 
static int e1000_init_receive_ring(int blocksize, int blockcount)
{
  rx_desc_ring = malloc(sizeof(struct e1000_rx_ring));
  if (!rx_desc_ring) {
    ERROR("Cannot allocate rx buffer\n");
    return -1;
  }
  rx_desc_ring->tail_pos = 0;
  
  //these are memory blocks that will store a packet
  rcv_block_size = blocksize; //should reflect register value (256)
  // the total size of the receive descriptor ring
  rd_buff_size = sizeof(struct e1000_rx_desc) * blockcount; //16Bytes *  ...
  // the number of the receive descriptor in the ring
  rcv_desc_count = blockcount;

  // allocate a large block of memory to store receiving packets
  rcv_buffer = malloc(rcv_block_size * rcv_desc_count);
  if (!rcv_buffer) {
    ERROR("Cannot allocate tx buffer\n");
    return -1;
  }

  //same 
  memset(rcv_buffer, 0, rcv_block_size * rcv_desc_count);
  // allocate the receive descriptor ring buffer
  rx_desc_ring->ring_buffer = malloc(rd_buff_size);
  if (!rx_desc_ring->ring_buffer) {
    ERROR("Cannot allocate rx buffer\n");
    return -1;
  }
  memset(rx_desc_ring->ring_buffer, 0, rd_buff_size);
 
  // initialize descriptors pointing to where device can write rcv'd packets
  for(int i=0; i<rcv_desc_count; i++)
  { e1000_init_single_rxd(i); }

  DEBUG("RX BUFFER AT %p\n",rx_desc_ring->ring_buffer); // we need this to be < 4GB

  // store the address of the memory in TDBAL/TDBAH 
  WRITE(vdev, RDBAL_OFFSET, (uint32_t)(0x00000000ffffffff & (uint64_t) rx_desc_ring->ring_buffer));
  WRITE(vdev, RDBAH_OFFSET, (uint32_t)((0xffffffff00000000 & (uint64_t) rx_desc_ring->ring_buffer) >> 32));
  DEBUG("rd_buffer=0x%016lx, RDBAL=0x%08x, RDBAH=0x%08x\n",
        rx_desc_ring->ring_buffer, READ(vdev, RDBAL_OFFSET), READ(vdev, RDBAH_OFFSET));
  // write rdlen
  WRITE(vdev, RDLEN_OFFSET, rd_buff_size);
  // write the rdh, rdt with 0
  WRITE(vdev, RDH_OFFSET, 0);
  WRITE(vdev, RDT_OFFSET, 0);
  rx_desc_ring->head_prev = 0;
  rx_desc_ring->tail_pos = 0;
  // write rctl register
  WRITE(vdev, RCTL_OFFSET, 0x0083832e);
  DEBUG("RDLEN=0x%08x, RDH=0x%08x, RDT=0x%08x, RCTL=0x%08x",
        READ(vdev, RDLEN_OFFSET), READ(vdev, RDH_OFFSET), READ(vdev, RDT_OFFSET),
        READ(vdev, RCTL_OFFSET));
  return 0;
}

// initialize ring buffer to hold transmit descriptors 
static int e1000_init_transmit_ring(int tx_dsc_count)
{
  td_buff_size = sizeof(struct e1000_tx_desc)*tx_dsc_count;
  // allocate transmit descriptor list ring buffer for 64kB.
  td_buffer = malloc(td_buff_size);
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
  return 0;
}

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
  d->addr = (uint64_t)packet_addr;
  d->length = packet_size;
  
  DEBUG("Sizeof e1000_tx_desc is %d\n",sizeof(struct e1000_tx_desc));

  WRITE(vdev, TDT_OFFSET, tdt+1); //increment transmit descriptor list tail by 1
  DEBUG("TDT=%d\n", READ(vdev, TDT_OFFSET));  
  DEBUG("TDH=%d\n", READ(vdev, TDH_OFFSET));
  DEBUG("e1000 status=0x%x\n", READ(vdev, 0x8));  
  return 0;
}

static void* e1000_receive_packet(uint64_t* dst_addr, int dst_size) {
  int headpos;
  int consumed;
  int index;
  int eop = 0;
  // make sure a full, fresh ring is available for packet
  for(int m = 0; m < RX_DSC_COUNT; m++){
      e1000_init_single_rxd(m);
  }
  rx_desc_ring->tail_pos = (rx_desc_ring->tail_pos + RX_DSC_COUNT-1) % RX_DSC_COUNT;
  WRITE(vdev, RDH_OFFSET, rx_desc_ring->tail_pos);

  // start listening
  while(1){
      headpos = READ(vdev, RDH_OFFSET);
      rx_desc_ring->tail_pos = READ(vdev, RDT_OFFSET);
      consumed = (RX_DSC_COUNT + headpos - rx_desc_ring->head_prev) % RX_DSC_COUNT;
      for(int i = 0; i < consumed; i++){
          // we move data from rcv_blocks to dst_block as rxd's are consumed
          index = (i + rx_desc_ring->head_prev) % RX_DSC_COUNT;
          // if descriptor done (dd), data has been copied into the block
          // corresponding to the descriptor
          if(RXD_STATUS(index).dd & 1){
              DEBUG("dd: %d\n", RXD_STATUS(index).dd);
              for(int j = 0; j < RXD_LENGTH(index)/64; j++){
                  //TODO check DMA atomic size; right now we assume it's multiples
                  //of 64B
                  dst_addr[j] = RXD_ADDR(index)[j];
              }
              // move the tail 
              rx_desc_ring->head_prev = (rx_desc_ring->head_prev + 1) % RX_DSC_COUNT;
              e1000_init_single_rxd(index);
              rx_desc_ring->tail_pos = (rx_desc_ring->tail_pos + 1) % RX_DSC_COUNT;
              WRITE(vdev, RDH_OFFSET, rx_desc_ring->tail_pos);
              // is end of packet (eop)?
              DEBUG("eop: %d\n", RXD_STATUS(index).eop);
              if(RXD_STATUS(index).eop & 1){
                  eop = 1;
              }
          }
      }
      if(eop){ 
          break;
      }
  }
  return (void*)0;
}

int e1000_get_characteristics(void *state, struct nk_net_dev_characteristics *c) {
  struct e1000_state *e=(struct e1000_state*)state;
  // c->mac=macaddress of the device from the state
  // min_tu the minimum pkt size to tx
  // c->min_tu =
  // c->max_tu =
  return 0; //succeeds
}

int e1000_interupt_handler() {
  // 1. figure which e1000 -> this gives the e1000 state
  // 2. figure why the interrupt error rx tx
  // 3. assume tx
  // 4. determine the position of the descriptor that is done
  // 5. use the data structure in e1000_post_send 3. to map the pos back to
  // callback and context
  // 6. run callback(context);
  return 0;
}
int e1000_post_send(void *state, uint8_t *src, uint64_t len, void (*callback)(void *context), void *context){
  struct e1000_state *e=(struct e1000_state*)state;
  // start the tx
  // check len < max_tu if no, return -1
  // queue full return -1
  // as simple as possible
  // 1. create the descriptor based on src (pkt address), len (length of the pkt)
  // 2. queue the descriptor in the hw and record the pos of the descriptor of
  // the ring
  // 3. put the data structure to map the pos in the ring from 2. to callback
  // and context (struct ...) [] -> callback and context
  // ring desc [ ] [ ] [ ]
  // array     [ ] [ ] [ ]
  return 0; //succeeds
}
int e1000_post_receive(void *state, uint8_t *src, uint64_t len, void (*callback)(void *context), void *context){
  // start the tx
  // 1. create the descriptor based on src (pkt address), len (length of the pkt)
  // 2. queue the descriptor in the hw and record the pos of the descriptor of
  // the ring
  // 3. put the data structure to map the pos in the ring from 2. to
  // callback and context
  return 0;
}

static struct nk_net_dev_int ops={
  .get_characteristics=e1000_get_characteristics,
  .post_receive=e1000_post_receive,
  .post_send=e1000_post_send,
};


int e1000_pci_init(struct naut_info * naut)
{
  struct pci_info *pci = naut->sys.pci;
  struct list_head *curbus, *curdev;
  int num = 0;
  
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
        char name[80];
        sprintf(name, "e1000-%d",num);
        num++;
        vdev->netdev = nk_net_dev_register(name, 0, &ops, vdev);
        if(!vdev->netdev) {
          ERROR("Cannot register the device");
        }
      }      
    }
  }
  // need to init each e1000
  e1000_init_transmit_ring(TX_DSC_COUNT);
  e1000_init_receive_ring(RX_BLOCKSIZE, RX_DSC_COUNT);
/*   return 0; // end of the function here */
/* } */
  // ***************** INIT IS COMPLETE ************************* //
  
  e1000_send_packet(my_packet, (uint16_t)sizeof(my_packet));
  e1000_send_packet(my_packet, (uint16_t)sizeof(my_packet));
  uint64_t* my_rcv_space = malloc(8*1024);
  DEBUG("receiving a packet");
  e1000_receive_packet(my_rcv_space, 8*1024);
    for(int j=0; j<42; j++)
    {
        DEBUG("index:%d %02x\n", j, *(uint8_t*)((uint8_t*)my_rcv_space + j));
        if(j%8 == 0)
        {
          DEBUG("byte: %d ----------------------------------------\n", j);
        }
    }
  DEBUG("tried to receive a packet");
  DEBUG("total pkt tx=%d\n", READ(vdev, TPT_OFFSET));
  DEBUG("total pkt tx=%d\n", READ(vdev, TPT_OFFSET));
  int sleepcount = 0;
  uint32_t headpos;
  uint32_t tailpos;
  uint32_t rstatus = 0;
  /*while(1)
  {
      if(sleepcount == 0) {
          headpos = READ(vdev, RDH_OFFSET);
          tailpos = READ(vdev, RDT_OFFSET);
          if (headpos == tailpos)
          {
              WRITE(vdev, RDT_OFFSET, (tailpos+1)%rcv_desc_count);
          }

          for(int i =0; i<rcv_desc_count; i++)
          {
              DEBUG("status of rd %d: %d\n", i, ((struct e1000_rx_desc*)
                                                 rx_desc_ring->ring_buffer)[i].status);
              if(((struct e1000_rx_desc*)rx_desc_ring->ring_buffer)[i].status.dd & 1)
              {
                    DEBUG("value is: %d, length is: %d\n",
                          *(uint64_t*)((uint8_t*)rcv_buffer+rcv_block_size*i),
                          ((struct e1000_rx_desc*)rx_desc_ring->ring_buffer)[i].length);
                    for(int j=0; j<((struct e1000_rx_desc*)rx_desc_ring->ring_buffer)[i].length; j++)
                    {
                        DEBUG("index:%d %02x\n", j, *(uint8_t*)((uint8_t*)rcv_buffer+rcv_block_size*i + j));
                        if(j%8 == 0)
                        {
                          DEBUG("byte: %d ----------------------------------------\n", j);
                        }
                    }
              }
          }
      } else if(sleepcount > 99999999999999999999){
          sleepcount = -1;
      }
      sleepcount++;
  }
*/

  return 0;
}

int e1000_pci_deinit()
{
  INFO("deinited\n");
  return 0;
}

