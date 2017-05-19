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

#include <nautilus/nautilus.h>
#include <nautilus/netdev.h>
#include <nautilus/cpu.h>
#include <dev/pci.h>
#include <nautilus/mm.h>              // malloc, free
#include <dev/e1000_pci.h>
#include <nautilus/irq.h>             // interrupt register
#include <nautilus/arp.h>             // dump_packet
#include <nautilus/naut_string.h>     // memset, memcpy
#include <nautilus/dev.h>             // NK_DEV_REQ_*

#ifndef NAUT_CONFIG_DEBUG_E1000_PCI
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...)
#endif

#define INFO(fmt, args...)           INFO_PRINT("E1000_PCI: " fmt, ##args)
#define DEBUG(fmt, args...)          DEBUG_PRINT("E1000_PCI: " fmt, ##args)
#define ERROR(fmt, args...)          ERROR_PRINT("E1000_PCI: " fmt, ##args)
#define READ(d, o)                   (*((volatile uint32_t*)(((d)->mem_start)+(o))))
#define READ_MEM(d, o)               (*((volatile uint32_t*)(((d)->mem_start)+(o))))
#define WRITE(d, o, v)               ((*((volatile uint32_t*)(((d)->mem_start)+(o))))=(v))
#define WRITE_MEM(d, o, v)           ((*((volatile uint32_t*)(((d)->mem_start)+(o))))=(v))

#define RXD_RING    (state->rx_ring) // e1000_ring *
#define RXD_PREV_HEAD (RXD_RING->head_prev)
#define RXD_TAIL (RXD_RING->tail_pos)
#define RX_PACKET_BUFFER   (RXD_RING->packet_buffer) // void *, packet buff addr
#define RXD_RING_BUFFER (RXD_RING->ring_buffer) // void *, ring buff addr
#define RX_DESC(i)     (((struct e1000_rx_desc*)RXD_RING_BUFFER)[(i)])
#define RXD_ERRORS(i) (((struct e1000_rx_desc*)RXD_RING_BUFFER)[(i)].errors)
#define RXD_STATUS(i)  (((struct e1000_rx_desc*)RXD_RING_BUFFER)[(i)].status)
#define RXD_LENGTH(i)  (((struct e1000_rx_desc*)RXD_RING_BUFFER)[(i)].length)
#define RXD_ADDR(i)  (((struct e1000_rx_desc*)RXD_RING_BUFFER)[(i)].addr) // block addr
#define RXD_COUNT (RXD_RING->count)
#define RXD_INC(a,b) (((a) + (b)) % RXD_RING->count) // modular increment rxd index
#define NEXT_RXD RXD_INC(RXD_RING->tail_pos, 1)
#define PREV_RXD RXD_INC(RXD_RING->tail_pos, RXD_COUNT - 1)

#define TXD_RING (state->tx_ring) // e1000_ring *
#define TXD_PREV_HEAD (TXD_RING->head_prev)
#define TXD_TAIL (TXD_RING->tail_pos)
#define TX_PACKET_BUFFER   (TXD_RING->packet_buffer) // void *, packet buff addr
#define TXD_RING_BUFFER (TXD_RING->ring_buffer) // void *, ring buff addr
#define TXD_CMD(i)  (((struct e1000_tx_desc*)TXD_RING_BUFFER)[(i)].cmd)
#define TXD_STATUS(i)  (((struct e1000_tx_desc*)TXD_RING_BUFFER)[(i)].status)
#define TXD_LENGTH(i)  (((struct e1000_tx_desc*)TXD_RING_BUFFER)[(i)].length)
#define TXD_ADDR(i)  (((struct e1000_tx_desc*)TXD_RING_BUFFER)[(i)].addr) // block addr
#define TXD_COUNT (TXD_RING->count)
#define TXD_INC(a,b) (((a) + (b)) % TXD_RING->count) // modular increment txd index
#define NEXT_TXD TXD_INC(TXD_RING->tail_pos, 1)
#define PREV_TXD TXD_INC(TXD_RING->tail_pos, TXD_COUNT - 1)

// a is old index, b is incr amount, c is queue size
#define INCRING(a, b, c) (((a)+(b)) % c)
#define TXMAP (state->tx_map)
#define RXMAP (state->rx_map)

#define IRQ_NUMBER       11

// transmitting buffer
static struct e1000_state *dev_state = NULL;

// init_single_txd and init_single_rxd both assume that we're only using
// continuous space in memory to form an "array" of blocks; this doesn't have to
// be the case. If we want to dynamically allocate blocks from arbitrary places,
// mess with these functions
static void e1000_init_single_txd(int index, struct e1000_state *state) {
  // initialize single descriptor pointing to where device can write rcv'd packet
  struct e1000_tx_desc tmp_txd;
  memset(&tmp_txd, 0, sizeof(struct e1000_tx_desc));
  tmp_txd.addr = (uint64_t*)((uint8_t*)TX_PACKET_BUFFER + TXD_RING->blocksize*index);
  ((struct e1000_tx_desc *)TXD_RING_BUFFER)[index] = tmp_txd;
}

static void e1000_init_single_rxd(uint64_t index, struct e1000_state *state) {
  // initialize single descriptor pointing to where device can write rcv'd packet
  struct e1000_rx_desc tmp_rxd;
  memset(&tmp_rxd, 0, sizeof(struct e1000_rx_desc));
  tmp_rxd.addr = (uint64_t*)((uint8_t*)RX_PACKET_BUFFER + RXD_RING->blocksize*index);
  ((struct e1000_rx_desc *)RXD_RING_BUFFER)[index] = tmp_rxd;
}

// initialize a ring buffer to hold receive descriptor
static int e1000_init_receive_ring(int blocksize,
                                   int rx_dsc_count,
                                   struct e1000_state *state) {
  RXMAP = malloc(sizeof(struct e1000_map_ring));
  if (!RXMAP) {
    ERROR("Cannot allocate rxmap\n");
    return -1;
  }
  memset(RXMAP, 0, sizeof(struct e1000_map_ring));

  RXMAP->ring = malloc(sizeof(struct e1000_fn_map) * rx_dsc_count);
  if (!RXMAP->ring) {
    ERROR("Cannot allocate rxmap->ring\n");
    return -1;
  }
  memset(RXMAP->ring, 0, sizeof(struct e1000_fn_map) * rx_dsc_count);
  RXMAP->ring_size = rx_dsc_count;

  RXD_RING = malloc(sizeof(struct e1000_ring));
  if (!RXD_RING) {
    ERROR("Cannot allocate rxd_ring buffer\n");
    return -1;
  }
  memset(RXD_RING, 0, sizeof(struct e1000_ring));
  RXD_RING->tail_pos = 0;

  //these are memory blocks that will store a packet
  RXD_RING->blocksize = blocksize; //should reflect register value (256)
  // the total size of the receive descriptor ring
  int rd_buff_size = sizeof(struct e1000_rx_desc) * rx_dsc_count;
  // the number of the receive descriptor in the ring
  RXD_COUNT = rx_dsc_count;

  // allocate a large block of memory to store receiving packets
  RX_PACKET_BUFFER = malloc(RXD_RING->blocksize * RXD_COUNT);
  if (!RX_PACKET_BUFFER) {
    ERROR("Cannot allocate RX_PACKET_BUFFER\n");
    return -1;
  }
  memset(RX_PACKET_BUFFER, 0, RXD_RING->blocksize * RXD_COUNT);

  // allocate the receive descriptor ring buffer
  RXD_RING_BUFFER = malloc(rd_buff_size);
  if (!RXD_RING_BUFFER) {
    ERROR("Cannot allocate RXD_RING_BUFFER\n");
    return -1;
  }
  memset(RXD_RING_BUFFER, 0, rd_buff_size);

  DEBUG("RX BUFFER AT %p\n",RXD_RING_BUFFER); // we need this to be < 4GB

  // store the address of the memory in TDBAL/TDBAH
  WRITE(state->dev, RDBAL_OFFSET,
        (uint32_t)(0x00000000ffffffff & (uint64_t) RXD_RING_BUFFER));
  WRITE(state->dev, RDBAH_OFFSET,
        (uint32_t)((0xffffffff00000000 & (uint64_t) RXD_RING_BUFFER) >> 32));
  DEBUG("rd_buffer=0x%016lx, RDBAL=0x%08x, RDBAH=0x%08x\n",
        RXD_RING_BUFFER, READ(state->dev, RDBAL_OFFSET),
        READ(state->dev, RDBAH_OFFSET));
  // write rdlen
  WRITE(state->dev, RDLEN_OFFSET, rd_buff_size);
  // write the rdh, rdt with 0
  WRITE(state->dev, RDH_OFFSET, 0);
  WRITE(state->dev, RDT_OFFSET, 0);
  RXD_PREV_HEAD = 0;
  RXD_TAIL = 0;
  // write rctl register
  // receive mode
  uint32_t rctl_reg = E1000_RCTL_EN | E1000_RCTL_SBP | E1000_RCTL_UPE | E1000_RCTL_LPE | E1000_RCTL_BAM | E1000_RCTL_PMCF | E1000_RCTL_RDMTS_HALF | E1000_RCTL_BSIZE_2048;
  // receive buffer threshold and size
  DEBUG("rctl_reg = 0x%08x, expected value: 0x%08x\n", rctl_reg, 0x0083832e);
  // WRITE_MEM(state->dev, RCTL_OFFSET, 0x0083832e);
  WRITE_MEM(state->dev, RCTL_OFFSET, rctl_reg);
  DEBUG("RDLEN=0x%08x, RDH=0x%08x, RDT=0x%08x, RCTL=0x%08x\n",
        READ_MEM(state->dev, RDLEN_OFFSET),
        READ_MEM(state->dev, RDH_OFFSET),
        READ_MEM(state->dev, RDT_OFFSET),
        READ_MEM(state->dev, RCTL_OFFSET));
  return 0;
}

// initialize ring buffer to hold transmit descriptors
static int e1000_init_transmit_ring(int blocksize,
                                    int tx_dsc_count,
                                    struct e1000_state *state) {
  TXMAP = malloc(sizeof(struct e1000_map_ring));
  if (!TXMAP) {
    ERROR("Cannot allocate txmap\n");
    return -1;
  }
  memset(TXMAP, 0, sizeof(struct e1000_map_ring));

  TXMAP->ring_size = tx_dsc_count;
  TXMAP->ring = malloc(sizeof(struct e1000_fn_map) * tx_dsc_count);
  if (!TXMAP->ring) {
    ERROR("Cannot allocate txmap->ring\n");
    return -1;
  }
  memset(TXMAP->ring, 0, sizeof(struct e1000_fn_map) * tx_dsc_count);

  TXD_RING = malloc(sizeof(struct e1000_ring));
  if (!TXD_RING) {
    ERROR("Cannot allocate TXD_RING\n");
    return -1;
  }
  memset(TXD_RING, 0, sizeof(struct e1000_ring));

  // allocate TX_DESC_COUNT transmit descriptors in the ring buffer.
  TXD_RING_BUFFER = malloc(sizeof(struct e1000_tx_desc)*tx_dsc_count);
  if (!TXD_RING_BUFFER) {
    ERROR("Cannot allocate TXD_RING_BUFFER\n");
    return -1;
  }
  memset(TXD_RING_BUFFER, 0, sizeof(struct e1000_tx_desc)*tx_dsc_count);
  TXD_RING->blocksize = blocksize; //should reflect register value (256)

  TXD_COUNT = tx_dsc_count;
  TX_PACKET_BUFFER = malloc(TXD_RING->blocksize * TXD_COUNT);
  if (!TX_PACKET_BUFFER) {
    ERROR("Cannot allocate TX_PACKET_BUFFER\n");
    return -1;
  }
  //these are memory blocks that will store a packet
  memset(TX_PACKET_BUFFER, 0, TXD_RING->blocksize * TXD_COUNT);
  DEBUG("TX RING BUFFER at 0x%p\n", TXD_RING_BUFFER);

  // store the address of the memory in TDBAL/TDBAH
  WRITE_MEM(state->dev, TDBAL_OFFSET,
            (uint32_t)( 0x00000000ffffffff & (uint64_t) TXD_RING_BUFFER));
  WRITE_MEM(state->dev, TDBAH_OFFSET,
            (uint32_t)((0xffffffff00000000 & (uint64_t) TXD_RING_BUFFER) >> 32));
  DEBUG("TXD_RING_BUFFER=0x%016lx, TDBAH=0x%08x, TDBAL=0x%08x\n",
        TXD_RING_BUFFER, READ_MEM(state->dev, TDBAH_OFFSET),
        READ_MEM(state->dev, TDBAL_OFFSET));
  // write tdlen: transmit descriptor length
  WRITE_MEM(state->dev, TDLEN_OFFSET, sizeof(struct e1000_tx_desc)*tx_dsc_count);
  // write the tdh, tdt with 0
  WRITE_MEM(state->dev, TDT_OFFSET, 0);
  WRITE_MEM(state->dev, TDH_OFFSET, 0);
  TXD_PREV_HEAD = 0;
  TXD_TAIL = 0;
  DEBUG("TDLEN = 0x%08x, TDH = 0x%08x, TDT = 0x%08x\n",
        READ_MEM(state->dev, TDLEN_OFFSET), READ_MEM(state->dev, TDH_OFFSET),
        READ_MEM(state->dev, TDT_OFFSET));
  // write tctl register
  WRITE_MEM(state->dev, TCTL_OFFSET,
            E1000_TCTL_EN | E1000_TCTL_PSP | E1000_TCTL_COLD_FD);
  // write tipg regsiter     00,00 0000 0110,0000 0010 00,00 0000 1010 = 0x0060200a
  // will be zero when emulating hardware
  WRITE_MEM(state->dev, TIPG_OFFSET,
            E1000_TIPG_IPGT | E1000_TIPG_IPGR1_IEEE8023 | E1000_TIPG_IPGR2_IEEE8023); 
  DEBUG("TCTL = 0x%08x, TIPG = 0x%08x\n",
        READ_MEM(state->dev, TCTL_OFFSET), READ_MEM(state->dev, TIPG_OFFSET));
  return 0;
}

static int e1000_send_packet(uint8_t* packet_addr,
                             uint64_t packet_size,
                             struct e1000_state *state) {
  DEBUG("e1000_send_packet fn\n");
  DEBUG("packet_addr 0x%p packet_size: %d\n", packet_addr, packet_size);
  TXD_TAIL = READ_MEM(state->dev, TDT_OFFSET);
  DEBUG("status before sending a packet: TDH = %d TDT = %d tail_pos = %d\n",
        READ_MEM(state->dev, TDH_OFFSET),
        READ_MEM(state->dev, TDT_OFFSET),
        TXD_TAIL);
  DEBUG("icr: 0x%08x\n", READ_MEM(state->dev, E1000_ICR_OFFSET));
  DEBUG("ims: 0x%08x\n", READ_MEM(state->dev, E1000_IMS_OFFSET));
  DEBUG("tpt total packet transmit: %d\n", READ_MEM(state->dev, E1000_TPT_OFFSET));
  DEBUG("packet_size: %d\n", packet_size);
  if(packet_size > MAX_TU) {
    ERROR("packet is too large.\n");
    return -1;
  }

  e1000_init_single_txd(TXD_TAIL, state); // make new descriptor
  TXD_ADDR(TXD_TAIL) = (uint64_t*) packet_addr;
  TXD_LENGTH(TXD_TAIL) = packet_size;
  // set up send flags
  TXD_CMD(TXD_TAIL).dext = 0;
  DEBUG("TXD_CMD(TXD_TAIL).dext %d\n", TXD_CMD(TXD_TAIL).dext);
  TXD_CMD(TXD_TAIL).vle = 0;
  DEBUG("TXD_CMD(TXD_TAIL).vle %d\n", TXD_CMD(TXD_TAIL).vle);
  TXD_CMD(TXD_TAIL).ifcs = 1;
  DEBUG("TXD_CMD(TXD_TAIL).ifcs %d\n", TXD_CMD(TXD_TAIL).ifcs);  
  // set the end of packet flag if this is the last fragment
  TXD_CMD(TXD_TAIL).eop = 1;
  DEBUG("TXD_CMD(TXD_TAIL).eop %d\n", TXD_CMD(TXD_TAIL).eop);    
  // interrupt delay enable
  // TXD_CMD(TXD_TAIL).ide = 1;
  DEBUG("TXD_CMD(TXD_TAIL).ide %d\n", TXD_CMD(TXD_TAIL).ide);    
  // report the status of the descriptor
  TXD_CMD(TXD_TAIL).rs = 1;
  DEBUG("TXD_CMD(TXD_TAIL).rs %d\n", TXD_CMD(TXD_TAIL).rs);    
  
  DEBUG("TXD_ADDR(TXD_TAIL) 0x%p\n", TXD_ADDR(TXD_TAIL));
  // increment transmit descriptor list tail by 1
  DEBUG("moving the tail\n");
  WRITE_MEM(state->dev, TDT_OFFSET, TXD_INC(1,TXD_TAIL));
  // TXD_TAIL = NEXT_TXD;
  DEBUG("address 0x%p length 0x%p val length 0x%08x\n",
        &(TXD_ADDR(TXD_TAIL)), &(TXD_LENGTH(TXD_TAIL)),
        *((uint64_t*)&(TXD_LENGTH(TXD_TAIL))));
  DEBUG("TXD_STATUS(TXD_TAIL).dd: %d\n", TXD_STATUS(TXD_TAIL).dd);
  DEBUG("status after moving tail: TDH = %d TDT = %d tail_pos = %d\n",
        READ_MEM(state->dev, TDH_OFFSET),
        READ_MEM(state->dev, TDT_OFFSET),
        TXD_TAIL);
  // DEBUG("icr: 0x%08x\n", READ_MEM(state->dev, E1000_ICR_OFFSET));
  // DEBUG("ims: 0x%08x\n", READ_MEM(state->dev, E1000_IMS_OFFSET));
  DEBUG("transmit error: %d\n",
        TXD_STATUS(TXD_PREV_HEAD).ec | TXD_STATUS(TXD_PREV_HEAD).lc);
  DEBUG("tpt total packet transmit: %d\n", READ_MEM(state->dev, E1000_TPT_OFFSET));
  DEBUG("e1000_send_packet fn end\n");
  return 0;
}

static int e1000_receive_packet(uint8_t* buffer,
                                uint64_t buffer_size,
                                struct e1000_state *state) {
  DEBUG("e1000 receive packet fn buffer = 0x%p, len = %lu\n", buffer, buffer_size);
  DEBUG("before moving tail head: %d, tail: %d\n",
        READ_MEM(state->dev, RDH_OFFSET),
        READ_MEM(state->dev, RDT_OFFSET));
  // if the buffer size is changed, 
  // let the network adapter know the new buffer size
  if(state->rx_buffer_size != buffer_size) {
    uint32_t rctl = READ_MEM(state->dev, RCTL_OFFSET) & E1000_RCTL_BSIZE_MASK;
    switch(buffer_size) {
      case E1000_RECV_BSIZE_256:
        rctl |= E1000_RCTL_BSIZE_256;
        break;
      case E1000_RECV_BSIZE_512:
        rctl |= E1000_RCTL_BSIZE_512;
        break;
      case E1000_RECV_BSIZE_1024:
        rctl |= E1000_RCTL_BSIZE_1024;
        break;
      case E1000_RECV_BSIZE_2048:
        rctl |= E1000_RCTL_BSIZE_2048;
        break;
      case E1000_RECV_BSIZE_4096:
        rctl |= E1000_RCTL_BSIZE_4096;
        break;
      case E1000_RECV_BSIZE_8192:
        rctl |= E1000_RCTL_BSIZE_8192;
        break;
      case E1000_RECV_BSIZE_16384:
        rctl |= E1000_RCTL_BSIZE_16384;
        break;
      default:
        ERROR("unmatch buffer size ");
        return -1;
    }
    WRITE_MEM(state->dev, RCTL_OFFSET, rctl);
    state->rx_buffer_size = buffer_size;
  }
        
  RXD_PREV_HEAD = READ_MEM(state->dev, RDH_OFFSET);
  RXD_TAIL = READ_MEM(state->dev, RDT_OFFSET);
  e1000_init_single_rxd(RXD_TAIL, state);
  RXD_ADDR(RXD_TAIL) = (uint64_t*) buffer;
  RXD_TAIL = RXD_INC(RXD_TAIL, 1);
  WRITE(state->dev, RDT_OFFSET, RXD_TAIL);
  DEBUG("after moving tail head: %d, prev_head: %d tail: %d\n",
        READ_MEM(state->dev, RDH_OFFSET), RXD_PREV_HEAD,
        READ_MEM(state->dev, RDT_OFFSET));
  
  DEBUG("end e1000 receive paket fn -----------------------\n");
  return 0;
}

uint64_t e1000_packet_size_to_buffer_size(uint64_t sz) {
  // round up the number to the buffer size with power of two
  // citation: https://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
  sz--;
  sz |= sz >> 1;
  sz |= sz >> 2;
  sz |= sz >> 4;
  sz |= sz >> 8;
  sz |= sz >> 16;
  sz |= sz >> 32;  
  sz++;
  // if the size is larger than the maximun buffer size, return the largest size.
  if(sz >= E1000_RECV_BSIZE_MAX) {
    return E1000_RECV_BSIZE_MAX;
  } else {
    return sz;
  }
}

int e1000_get_characteristics(void *vstate, struct nk_net_dev_characteristics *c) {
  struct e1000_state *state=(struct e1000_state*)vstate;
  memcpy(c->mac, (void *) state->mac_addr, MAC_LEN);
  // minimum and the maximum transmission unit
  c->min_tu = MIN_TU; 
  c->max_tu = MAX_TU;
  c->packet_size_to_buffer_size = e1000_packet_size_to_buffer_size;
  return 0;
}

static void e1000_unmap_callback(struct e1000_map_ring* map,
                                 uint64_t** callback,
                                 void** context) {
  // callback is a function pointer
  DEBUG("unmap callback fn head_pos %d tail_pos %d\n", map->head_pos, map->tail_pos);
  if(map->head_pos == map->tail_pos) {
    // if there is an empty mapping ring buffer, do not unmap the callback
    ERROR("try to unmap an empty queue\n");
    return;
  }

  uint64_t i = map->head_pos;
  *callback = (uint64_t *) map->ring[i].callback;
  *context =  map->ring[i].context;
  map->ring[i].callback = NULL;
  map->ring[i].context = NULL;
  map->head_pos = (1 + map->head_pos) % map->ring_size;
  DEBUG("unmap callback fn head_pos %d tail_pos %d\n", map->head_pos, map->tail_pos);
}

static int e1000_map_callback(struct e1000_map_ring* map,
                              void (*callback)(nk_net_dev_status_t, void*),
                              void* context) {
  DEBUG("map callback head_pos %d tail_pos %d\n", map->head_pos, map->tail_pos);
  if(map->head_pos == ((map->tail_pos + 1) % map->ring_size)) {
    // when the mapping callback queue is full
    ERROR("Callback mapping queue is full.\n");
    return -1;
  }

  uint64_t i = map->tail_pos;
  struct e1000_fn_map *fnmap = (map->ring + i);
  fnmap->callback = callback;
  fnmap->context = (uint64_t *)context;
  map->tail_pos = (1 + map->tail_pos) % map->ring_size;
  DEBUG("mapped callback head_pos: %d, tail_pos: %d\n", map->head_pos, map->tail_pos);
  return 0;
}

int e1000_post_send(void *state,
                    uint8_t *src,
                    uint64_t len,
                    void (*callback)(nk_net_dev_status_t, void *),
                    void *context) {
  // always map callback
  int result = 0;
  DEBUG("post send fn callback 0x%p\n", callback);
  result = e1000_map_callback(((struct e1000_state*)state)->tx_map, callback, context);

  if (!result)
    // delete the context from the argument of this function
    result = e1000_send_packet(src, len, (struct e1000_state*) state);
  DEBUG("post end fn end\n");
  return result;
}

int e1000_post_receive(void *state,
                       uint8_t *src,
                       uint64_t len,
                       void (*callback)(nk_net_dev_status_t, void *),
                       void *context) {
  // mapping the callback always
  // if result != -1 receive packet
  int result = 0;
  DEBUG("post receive fn callback 0x%p\n", callback);
  result  = e1000_map_callback(((struct e1000_state*)state)->rx_map, callback, context);

  if(!result) 
    result = e1000_receive_packet(src, len, (struct e1000_state*) state);
  return result;
}

static int e1000_irq_handler(excp_entry_t * excp, excp_vec_t vec) {
  DEBUG("e1000_irq_handler fn vector: 0x%x rip: 0x%p\n", vec, excp->rip);
  uint32_t icr = READ_MEM(dev_state->dev, E1000_ICR_OFFSET);
  uint32_t ims = READ_MEM(dev_state->dev, E1000_IMS_OFFSET);
  uint32_t mask_int = icr & ims;
  DEBUG("ICR: 0x%08x IMS: 0x%08x mask_int: 0x%08x\n", icr, ims, mask_int);
  DEBUG("ICR: 0x%08x\n", READ_MEM(dev_state->dev, E1000_ICR_OFFSET));
  struct e1000_state* state = dev_state;
  
  void (*callback)(nk_net_dev_status_t, void*) = NULL;
  void *context = NULL;
  nk_net_dev_status_t status = NK_NET_DEV_STATUS_SUCCESS;
  
  if(mask_int & E1000_ICR_TXDW) {
    // transmit interrupt
    DEBUG("handle the txdw interrupt\n");
    e1000_unmap_callback(dev_state->tx_map, (uint64_t **)&callback, (void **)&context);
    // if there is an error while sending a packet, set the error status
    if(TXD_STATUS(TXD_PREV_HEAD).ec || TXD_STATUS(TXD_PREV_HEAD).lc) {
      ERROR("transmit errors\n");
      status = NK_NET_DEV_STATUS_ERROR;
    }
    // TODO: which one
    // TXD_PREV_HEAD = TXD_INC(1, TXD_PREV_HEAD);
    // TXD_TAIL = NEXT_TXD;
  } else if(mask_int & E1000_ICR_RXT0) {
    // receive interrupt
    DEBUG("handle the rxt0 interrupt\n");
    e1000_unmap_callback(dev_state->rx_map, (uint64_t **)&callback, (void **)&context);
    // checking errors
    if(RXD_ERRORS(RXD_PREV_HEAD)) {
      ERROR("receive an error packet\n");
      status = NK_NET_DEV_STATUS_ERROR;
    }

    RXD_PREV_HEAD = RXD_INC(1, RXD_PREV_HEAD);    
    DEBUG("total packet received = %d\n", READ_MEM(state->dev, E1000_TPR_OFFSET));
  }

  if(callback) {
    DEBUG("invoke callback function callback: 0x%p\n", callback);
    callback(status, context);
  }
  
  DEBUG("end irq\n\n\n");
  // must have this line at the end of the handler
  IRQ_HANDLER_END();
  return 0;
}

static struct nk_net_dev_int ops = {
  .get_characteristics = e1000_get_characteristics,
  .post_receive        = e1000_post_receive,
  .post_send           = e1000_post_send,
};

int e1000_pci_init(struct naut_info * naut) {
  struct e1000_state *state = (struct e1000_state *)malloc(sizeof(struct e1000_state));
  dev_state = state;
  struct pci_info *pci = naut->sys.pci;
  struct list_head *curbus, *curdev;
  int num = 0;

  INFO("init\n");
  // building the e1000 linked list and register it
  // linked list of e1000 devices
  struct list_head dev_list;
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
      if (cfg->vendor_id==INTEL_VENDOR_ID && cfg->device_id==E1000_DEVICE_ID) {
        DEBUG("E1000 Device Found\n");
        // struct e1000_dev *dev;
        struct e1000_dev *dev = malloc(sizeof(struct e1000_dev));
        if (!dev) {
          ERROR("Cannot allocate device\n");
          return -1;
        }

        memset(dev,0,sizeof(*dev));
        state->dev = dev;
        dev->pci_dev = pdev;

        // PCI Interrupt (A..D)
        dev->pci_intr = cfg->dev_cfg.intr_pin;
        // Figure out mapping here or look at capabilities for MSI-X
        // dev->intr_vec = ... (for future work)

        // find out the bar for e1000
        for (int i=0;i<6;i++) {
          uint32_t bar = pci_cfg_readl(bus->num, pdev->num, 0, 0x10 + i*4);
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
          pci_cfg_writel(bus->num, pdev->num, 0, 0x10 + i*4, 0xffffffff);
          // size mask comes back + info bits
          // write all ones and read back. if we get 00 (negative size), size = 4.
          size = pci_cfg_readl(bus->num, pdev->num, 0, 0x10 + i*4);

          // mask all but size mask
          if (bar & 0x1) { // I/O
            size &= 0xfffffffc;
          } else { // memory
            size &= 0xfffffff0;
          }
          // two complement, get back the positive size
          size = ~size;
          size++;

          // now we have to put back the original bar
          pci_cfg_writel(bus->num, pdev->num, 0, 0x10 + i*4, bar);

          if (!size) { // size = 0 -> non-existent bar, skip to next one
            continue;
          }

          uint32_t start;
          if (bar & 0x1) {
            start = dev->ioport_start = bar & 0xffffffc0;
            dev->ioport_end = dev->ioport_start + size;
          } else {
            start = dev->mem_start = bar & 0xfffffff0;
            dev->mem_end = dev->mem_start + size;
          }

          DEBUG("bar %d is %s address=0x%x size=0x%x\n", i,
                bar & 0x1 ? "io port":"memory", start, size);
        }

        INFO("Adding e1000 device: bus=%u dev=%u func=%u: pci_intr=%u intr_vec=%u ioport_start=%p ioport_end=%p mem_start=%p mem_end=%p\n",
             bus->num, pdev->num, 0,
             dev->pci_intr, dev->intr_vec,
             dev->ioport_start, dev->ioport_end,
             dev->mem_start, dev->mem_end);

        uint16_t old_cmd = pci_cfg_readw(bus->num,pdev->num,0,0x4);
        DEBUG("Old PCI CMD: %x\n",old_cmd);

        old_cmd |= 0x7;  // make sure bus master is enabled
        old_cmd &= ~0x40;

        DEBUG("New PCI CMD: %x\n",old_cmd);

        pci_cfg_writew(bus->num,pdev->num,0,0x4,old_cmd);

        uint16_t stat = pci_cfg_readw(bus->num,pdev->num,0,0x6);
        DEBUG("PCI STATUS: %x\n",stat);

        // read the status register at void ptr + offset
        uint32_t status = READ_MEM(state->dev, E1000_STATUS_OFFSET);
        DEBUG("e1000 status = 0x%x\n", status);
        uint32_t mac_low = READ_MEM(state->dev, E1000_RAL_OFFSET);
        uint32_t mac_high = READ_MEM(state->dev, E1000_RAH_OFFSET);
        uint64_t mac_all = ((uint64_t)mac_low+((uint64_t)mac_high<<32)) & 0xffffffffffff;
        DEBUG("e1000 mac_all = 0x%lX\n", mac_all);
        DEBUG("e1000 mac_high = 0x%x mac_low = 0x%x\n", mac_high, mac_low);
        memcpy(state->mac_addr, &mac_all, MAC_LEN);

        list_add(&dev_list, &dev->e1000_node);
        sprintf(state->name, "e1000-%d", num);
        num++;
        dev->netdev = nk_net_dev_register(state->name, 0, &ops, (void *)state);
        if(!dev->netdev) {
          ERROR("Cannot register the device");
          return -1;
        }
      }
    }
  }

  if(!state->dev) {
    free(state);
    state = NULL;
    ERROR("cannot find the e1000 device");
    return 0;
  }

  // register the interrupt handler
  register_irq_handler(IRQ_NUMBER, e1000_irq_handler, NULL);
  nk_unmask_irq(IRQ_NUMBER);
  // interrupt delay value = 0 -> does not delay
  WRITE_MEM(state->dev, E1000_TIDV_OFFSET, 0);
  // receive interrupt delay timer = 0
  // -> interrupt when the device receives a package
  WRITE_MEM(state->dev, E1000_RDTR_OFFSET, 0);
  // enable only transmit descriptor written back and receive interrupt timer
  WRITE_MEM(state->dev, E1000_IMS_OFFSET, E1000_ICR_TXDW | E1000_ICR_RXT0);

  uint32_t status = 0;
  status = READ_MEM(state->dev, E1000_IMS_OFFSET);
  DEBUG("IMS: %d\n", status);
  status = READ_MEM(state->dev, E1000_ICR_OFFSET);
  DEBUG("ICR = 0x%08x TXQE 0x%08x TXD_LOW 0x%08x TXDW 0x%08x\n",
        E1000_ICR_TXQE & status, E1000_ICR_TXD_LOW & status, E1000_ICR_TXDW & status);
  e1000_init_transmit_ring(TX_BLOCKSIZE, TX_DSC_COUNT, state);
  e1000_init_receive_ring(RX_BLOCKSIZE, RX_DSC_COUNT, state);
  return 0;
}

int e1000_pci_deinit() {
  INFO("deinited\n");
  return 0;
}
