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

uint32_t my_packet[16] = {0xdeadbeef,0xbeefdead, 0x12345678, 0x87654321, };

// init_single_txd and init_single_rxd both assume that we're only using
// continuous space in memory to form an "array" of blocks; this doesn't have to
// be the case. If we want to dynamically allocate blocks from arbitrary places,
// mess with these functions
static void e1000_init_single_txd(int index, struct e1000_state *state){
  // initialize single descriptor pointing to where device can write rcv'd packet
  struct e1000_tx_desc tmp_txd;
  memset(&tmp_txd, 0, sizeof(struct e1000_tx_desc));
  tmp_txd.addr = (uint64_t)((uint8_t*)TX_PACKET_BUFFER + TXD_RING->blocksize*index);
  ((struct e1000_tx_desc *)TXD_RING_BUFFER)[index] = tmp_txd;
  TXD_CMD(TXD_TAIL).dext = 0;
  TXD_CMD(TXD_TAIL).vle = 0;
  return;
}

static int e1000_init_single_rxd(int index, struct e1000_state *state){
  // initialize single descriptor pointing to where device can write rcv'd packet
  struct e1000_rx_desc tmp_rxd;
  memset(&tmp_rxd, 0, sizeof(struct e1000_rx_desc));
  tmp_rxd.addr = (uint64_t)((uint8_t*)RX_PACKET_BUFFER + RXD_RING->blocksize*index);
  ((struct e1000_rx_desc *)RXD_RING_BUFFER)[index] = tmp_rxd;
  return 0;
}

// initialize a ring buffer to hold receive descriptor and
// another buffer for DMA space
static int e1000_init_receive_ring(int blocksize, int rx_dsc_count, struct e1000_state *state)
{
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

  // initialize descriptors pointing to where device can write rcv'd packets
  for(int i=0; i<RXD_COUNT; i++)
  { e1000_init_single_rxd(i, state); }

  DEBUG("RX BUFFER AT %p\n",RXD_RING_BUFFER); // we need this to be < 4GB

  // store the address of the memory in TDBAL/TDBAH
  WRITE(state->dev, RDBAL_OFFSET, (uint32_t)(0x00000000ffffffff & (uint64_t) RXD_RING_BUFFER));
  WRITE(state->dev, RDBAH_OFFSET, (uint32_t)((0xffffffff00000000 & (uint64_t) RXD_RING_BUFFER) >> 32));
  DEBUG("rd_buffer=0x%016lx, RDBAL=0x%08x, RDBAH=0x%08x\n",
        RXD_RING_BUFFER, READ(state->dev, RDBAL_OFFSET), READ(state->dev, RDBAH_OFFSET));
  // write rdlen
  WRITE(state->dev, RDLEN_OFFSET, rd_buff_size);
  // write the rdh, rdt with 0
  WRITE(state->dev, RDH_OFFSET, 0);
  WRITE(state->dev, RDT_OFFSET, 0);
  RXD_PREV_HEAD = 0;
  RXD_TAIL = 0;
  // write rctl register
  WRITE_MEM(state->dev, RCTL_OFFSET, 0x0083832e);
  DEBUG("RDLEN=0x%08x, RDH=0x%08x, RDT=0x%08x, RCTL=0x%08x\n",
        READ_MEM(state->dev, RDLEN_OFFSET),
        READ_MEM(state->dev, RDH_OFFSET),
        READ_MEM(state->dev, RDT_OFFSET),
        READ_MEM(state->dev, RCTL_OFFSET));
  return 0;
}

// initialize ring buffer to hold transmit descriptors
static int e1000_init_transmit_ring(int blocksize, int tx_dsc_count,
                                    struct e1000_state *state)
{
  TXMAP = malloc(sizeof(struct e1000_map_ring));
  if (!TXMAP) {
    ERROR("Cannot allocate txmap\n");
    return -1;
  }
  memset(TXMAP, 0, sizeof(struct e1000_map_ring));

  TXMAP->ring_size = tx_dsc_count;
  DEBUG("tx dsc count: %d TXMAP->ring_size: %d\n",
        tx_dsc_count, TXMAP->ring_size);
  TXMAP->ring = malloc(sizeof(struct e1000_fn_map) * tx_dsc_count);
  if (!TXMAP->ring) {
    ERROR("Cannot allocate txmap->ring\n");
    return -1;
  }
  memset(TXMAP->ring, 0, sizeof(struct e1000_fn_map) * tx_dsc_count);
  DEBUG("tx dsc count: %d TXMAP->ring_size: %d\n",
        tx_dsc_count, TXMAP->ring_size);
  
  TXD_RING = malloc(sizeof(struct e1000_ring));
  if (!TXD_RING) {
    ERROR("Cannot allocate TXD_RING\n");
    return -1;
  }
  memset(TXD_RING, 0, sizeof(struct e1000_ring));
  
  DEBUG("tx dsc count: %d TXMAP->ring_size: %d\n",
        tx_dsc_count, TXMAP->ring_size);
  int td_buff_size = sizeof(struct e1000_tx_desc)*tx_dsc_count;
  // allocate transmit descriptor list ring buffer for 64kB.
  TXD_RING_BUFFER = malloc(td_buff_size);
  if (!TXD_RING_BUFFER) {
    ERROR("Cannot allocate TXD_RING_BUFFER\n");
    return -1;
  }
  memset(TXD_RING_BUFFER, 0, td_buff_size);
  TXD_RING->blocksize = blocksize; //should reflect register value (256)
  DEBUG("tx dsc count: %d TXMAP->ring_size: %d\n",
        tx_dsc_count, TXMAP->ring_size);
  TXD_COUNT = tx_dsc_count;
  TX_PACKET_BUFFER = malloc(TXD_RING->blocksize * TXD_COUNT);
  if (!TX_PACKET_BUFFER) {
    ERROR("Cannot allocate TX_PACKET_BUFFER\n");
    return -1;
  }
  //same
  memset(TX_PACKET_BUFFER, 0, TXD_RING->blocksize * TXD_COUNT);
  DEBUG("tx dsc count: %d TXMAP->ring_size: %d\n",
        tx_dsc_count, TXMAP->ring_size);
  //these are memory blocks that will store a packet

  DEBUG("TX BUFFER AT %p\n",TXD_RING_BUFFER); // we need this to be < 4GB

  // store the address of the memory in TDBAL/TDBAH
  WRITE(state->dev, TDBAL_OFFSET,
        (uint32_t)(0x00000000ffffffff & (uint64_t) TXD_RING_BUFFER));
  WRITE(state->dev, TDBAH_OFFSET,
        (uint32_t)((0xffffffff00000000 & (uint64_t) TXD_RING_BUFFER) >> 32));
  DEBUG("TXD_RING_BUFFER=0x%016lx, TDBAL=0x%08x, TDBAH=0x%08x\n",
        TXD_RING_BUFFER, READ(state->dev, TDBAL_OFFSET),
        READ(state->dev, TDBAH_OFFSET));
  // write tdlen: transmit descriptor length
  WRITE(state->dev, TDLEN_OFFSET, 64*1024);
  // write the tdh, tdt with 0
  WRITE(state->dev, TDT_OFFSET, 0);
  WRITE(state->dev, TDH_OFFSET, 0);
  TXD_PREV_HEAD = 0;
  TXD_TAIL = 0;
  // write tctl register
  WRITE(state->dev, TCTL_OFFSET, 0x4010a);
  // write tipg regsiter     00,00 0000 0110,0000 0010 00,00 0000 1010
  WRITE(state->dev, TIPG_OFFSET, 0x0060200a); // will be zero when emulating hardware
  DEBUG("TDLEN=0x%08x, TDH=0x%08x, TDT=0x%08x, TCTL=0x%08x, TIPG=0x%08x\n",
        READ(state->dev, TDLEN_OFFSET), READ(state->dev, TDH_OFFSET),
        READ(state->dev, TDT_OFFSET),
        READ(state->dev, TCTL_OFFSET), READ(state->dev, TIPG_OFFSET));
  return 0;
}

static int e1000_send_packet(void* packet_addr, uint16_t packet_size,
                             struct e1000_state *state, void* context) {
  /* uint16_t remaining_bytes = packet_size; */
  /* uint64_t unresolved = 0; */
  /* uint64_t oldest = TXD_TAIL; */
  /* uint64_t index; */
  /* uint8_t eop = 0; */

  /* if(TXD_RING->blocksize < packet_size) */
  /*   return -1; */
  /* // loop until 1) all bytes have been pushed to descriptors */
  /* //      and   2) all used descriptors have been marked as done (sent) */
  /* DEBUG("TDH=%d\n", READ(state->dev, TDH_OFFSET)); */
  /* DEBUG("TDT=%d\n", READ(state->dev, TDT_OFFSET));   */
  /* DEBUG("td buffer=%d tail_pos: %d\n", TXD_RING_BUFFER, TXD_TAIL); */
  /* while(remaining_bytes > 0 || unresolved != 0) { */
  /*   DEBUG("TDT=%d\n", TXD_TAIL); */
  /*   if(unresolved) { */
  /*     // see if any not previously done txd's are now done */
  /*     for(index = oldest; index != TXD_TAIL; index++){ */
  /*       if(TXD_STATUS(index).dd & 1){ */
  /*         oldest = TXD_INC(index, 1); */
  /*         unresolved--; */
  /*       } else { */
  /*         break; */
  /*       } */
  /*     } */
  /*   } */

  /*   // if there is more data and the txd is not full */
  /*   if(remaining_bytes > 0 && READ(state->dev, TDH_OFFSET) != NEXT_TXD) {  */
  /*     // push as many remaining bytes as possible onto a new txd's block */
  /*     e1000_init_single_txd(TXD_TAIL, state); // make new descriptor */
  /*     uint64_t cpy_size = remaining_bytes > TXD_RING->blocksize ? TXD_RING->blocksize : remaining_bytes; */
  /*     memcpy((void*)TXD_ADDR(TXD_TAIL), packet_addr, cpy_size); */
  /*     remaining_bytes -= cpy_size; */
  /*     // set up send flags */
  /*     TXD_CMD(TXD_TAIL).dext = 0; */
  /*     TXD_CMD(TXD_TAIL).vle = 0; */
  /*     // set the end of packet flag if this is the last fragment */
  /*     if(remaining_bytes == 0) { */
  /*       TXD_CMD(TXD_TAIL).ifcs = 1; */
  /*       TXD_CMD(TXD_TAIL).eop = 1; */
  /*     } else { */
  /*       TXD_CMD(TXD_TAIL).ifcs = 0; */
  /*       TXD_CMD(TXD_TAIL).eop = 0;         */
  /*     } */

  /*     TXD_CMD(TXD_TAIL).ide = 1; */
  /*     TXD_CMD(TXD_TAIL).rs = 1; */
  /*     TXD_LENGTH(TXD_TAIL) = cpy_size; */

  /*     DEBUG("packet_size: %d, remaining_bytes: %d block_size: %d cpy_size: %d\n", */
  /*           packet_size, remaining_bytes, TXD_RING->blocksize, cpy_size); */
  /*     // increment transmit descriptor list tail by 1 */
  /*     WRITE(state->dev, TDT_OFFSET, NEXT_TXD); */
  /*     unresolved++; // note that there is a new not done txd */
  /*     TXD_TAIL = NEXT_TXD; */
  /*     DEBUG("TDH=%d\n", READ(state->dev, TDH_OFFSET)); */
  /*     DEBUG("TDT=%d\n", READ(state->dev, TDT_OFFSET)); */
  /*     DEBUG("e1000 status=0x%x\n", READ(state->dev, E1000_STATUS_OFFSET)); */
  /*   } */
  /* } */
  /* DEBUG("DONE SENT -----------------------\n"); */
  /* return 0; */

  if(TXD_RING->blocksize < packet_size) {
    ERROR("cannot send a packet: the packet_size %d is larger than blocksize %d\n",
          packet_size, TXD_RING->blocksize);
    return -1;
  }

  DEBUG("tx TDH = %d, TDT = %d\n",
        READ(state->dev, TDH_OFFSET), READ(state->dev, TDT_OFFSET));
  e1000_init_single_txd(TXD_TAIL, state); // make new descriptor
  memcpy((void*)TXD_ADDR(TXD_TAIL), packet_addr, packet_size);
  TXD_LENGTH(TXD_TAIL) = packet_size;
  TXD_CMD(TXD_TAIL).ifcs = 1;
  TXD_CMD(TXD_TAIL).eop = 1;

  if(context) {
    // blocking
    TXD_CMD(TXD_TAIL).ide = 1;
  } else {
    // non-blocking 
    TXD_CMD(TXD_TAIL).ide = 0;
  }

  TXD_CMD(TXD_TAIL).rs = 1;

  // move the tail, and update it
  TXD_TAIL = NEXT_TXD;
  DEBUG("moving the tail\n");
  WRITE(state->dev, TDT_OFFSET, TXD_TAIL);
  DEBUG("TDH = %d, TDT = %d, TXD_STATUS(TXD_TAIL-1).dd = %d\n",
        READ(state->dev, TDH_OFFSET), READ(state->dev, TDT_OFFSET),
        TXD_STATUS(TXD_TAIL-1).dd);
  while(context && TXD_STATUS(TXD_TAIL-1).dd == 0) {
    DEBUG("polling\n");
  }
  DEBUG("DONE send -----\n");
  return 0;
}

static int e1000_receive_packet(uint64_t* dst_addr, uint64_t dst_size, struct e1000_state *state) {
  // TODO we're careless about the dst_size; there should probably be some kind of
  // check to make sure we're not going past any bounds as we write
  uint32_t headpos;
  uint8_t eop = 0;
  uint32_t j;
  uint32_t index = 0;
  DEBUG("Start receive, dest=%p, len=%lu\n", dst_addr, dst_size);
  RXD_TAIL = READ(state->dev, RDT_OFFSET);
  e1000_init_single_rxd(NEXT_RXD, state);
  RXD_TAIL = NEXT_RXD;
  RXD_PREV_HEAD = READ(state->dev, RDH_OFFSET);
  WRITE(state->dev, RDT_OFFSET, RXD_TAIL);

  // start listening
  while(!(eop)){
    headpos = READ(state->dev, RDH_OFFSET);
    while(headpos != RXD_PREV_HEAD){
      if(RXD_STATUS(RXD_PREV_HEAD).dd & 1){
        DEBUG("dd: %d\n", RXD_STATUS(RXD_PREV_HEAD).dd);
        DEBUG("RXD_LENGTH(RXD_PREV_HEAD): %d\n", RXD_LENGTH(RXD_PREV_HEAD));
        for(j = 0; j < RXD_LENGTH(RXD_PREV_HEAD); j++){
          //TODO check DMA atomic size; right now we assume it's multiples of 64B
          dst_addr[index+j] = ((uint64_t *)(RXD_ADDR(RXD_PREV_HEAD)))[j];
        }
        index += j;
        if(RXD_STATUS(RXD_PREV_HEAD).eop & 1){
          eop = 1;
        }
        DEBUG("eop: %d\n", RXD_STATUS(RXD_PREV_HEAD).eop);
        RXD_PREV_HEAD = RXD_INC(RXD_PREV_HEAD, 1);
      }
    }

    for(int m = 0; !eop && m < RX_DSC_COUNT && RXD_INC(RXD_TAIL, m+1) != RXD_PREV_HEAD; m++){
      e1000_init_single_rxd(RXD_INC(RXD_TAIL, m), state);
    }
    WRITE(state->dev, RDT_OFFSET, RXD_TAIL);
  }
  return 0;
}

int e1000_get_characteristics(void *vstate, struct nk_net_dev_characteristics *c) {
  struct e1000_state *state=(struct e1000_state*)vstate;
  memcpy((void *) &state->mac_addr, c->mac, MAC_LEN);
  // min_tu the minimum pkt size to tx
  c->min_tu = MIN_TU; // the device documentation seemed to imply a number like this
  c->max_tu = MAX_TU; // TODO we just picked this value out of the air; probably should find the real value
  return 0; //succeeds
}

static inline void e1000_unmap_callback(struct e1000_map_ring* map, 
                                        uint64_t** callback, void** context) {
  // callback is a function pointer
  DEBUG("unmap callback head_pos %d tail_pos %d\n", map->head_pos, map->tail_pos);
  uint64_t i = map->head_pos;
  struct e1000_fn_map fnmap = *(map->ring + i);
  // DEBUG("fnmap %p\n", fnmap);
  DEBUG("callback: %p\n", fnmap.callback);
  // DEBUG("callback 0x%p\n", map->ring[i].callback);  
  *callback = (uint64_t *) map->ring[i].callback;
  *context =  map->ring[i].context;
  if(*callback && *context) {
    map->ring[i].callback = NULL;
    map->ring[i].context = NULL;
    map->head_pos = (1 + map->head_pos) % map->ring_size;
  }
}

static inline void e1000_map_callback(struct e1000_map_ring* map, 
                                      void (*callback)(void*),
                                      void* context) {
  DEBUG("map callback head_pos %d tail_pos %d\n", map->head_pos, map->tail_pos);
  uint64_t i = map->tail_pos;
  struct e1000_fn_map *fnmap = (map->ring + i);
  fnmap->callback = callback;
  DEBUG("callback 0x%p\n", fnmap->callback);  
  fnmap->context = (uint64_t *)context;
  map->tail_pos = (1 + map->tail_pos) % map->ring_size;
}

int e1000_post_send(void *state, uint8_t *src, uint64_t len, void (*callback)(void *context), void *context){
  // TODO do interrupts
  // send right away and then do a callback when the send is done)
  // start the tx
  // check len < max_tu if no, return -1
  // queue full return -1
  // as simple as possible
  // 1. create the descriptor based on src (pkt address), len (length of the pkt)
  // 2. queue the descriptor in the hw and record the pos of the descriptor of
  // the ring
  // 3. put the data structure to map the pos in the ring from 2. to callback
  // and context (struct ...) [] -> callback and context
  int res = 0;
  if(callback) {
    // blocking
    DEBUG("blocking send callback 0x%p\n", callback);    
    e1000_map_callback(((struct e1000_state*)state)->tx_map, callback, context);
  } else {
    // non-blocking
    DEBUG("non-blocking send\n");
  }
  res = e1000_send_packet(src, (uint16_t) len, (struct e1000_state*) state, context);
  return res;
}

int e1000_post_receive(void *state, uint8_t *src, uint64_t len, void (*callback)(void *context), void *context) {
  // TODO (we don't do any queueing because we didn't do interrupts
  // we just try to send right away
  // and then do a callback when the send is done)
  // start the tx
  // 1. create the descriptor based on src (pkt address), len (length of the pkt)
  // 2. queue the descriptor in the hw and record the pos of the descriptor of
  // the ring
  // 3. put the data structure to map the pos in the ring from 2. to
  // callback and context

  int res = 0;
  if(callback) {
    // blocking
    DEBUG("blocking send callback 0x%p\n", callback);    
    e1000_map_callback(((struct e1000_state*)state)->tx_map, callback, context);
  } else {
    // non-blocking
    DEBUG("non-blocking send\n");
  }
  res = e1000_receive_packet(src, (uint16_t) len, (struct e1000_state*) state, context);
  return res;
}

static int e1000_irq_handler(excp_entry_t * excp, excp_vec_t vec)
{
  DEBUG("e1000_irq_handler vector: 0x%x rip: 0x%p\n", vec, excp->rip);
  // 1. figure which e1000 -> this gives the e1000 state
  //    need to add the third argument for this function
  // 2. figure why the interrupt error rx tx
  // 3. assume tx
  // 4. determine the position of the descriptor that is done
  // 5. use the data structure in e1000_post_send 3. to map the pos back to
  // callback and context
  // 6. run callback(context);
  uint32_t icr = READ_MEM(dev_state->dev, E1000_ICR_OFFSET);
  uint32_t ims = READ_MEM(dev_state->dev, E1000_IMS_OFFSET);
  uint32_t mask_int = icr & ims;
  DEBUG("ICR: 0x%08x IMS: 0x%08x mask_int: 0x%08x\n", icr, ims, mask_int);
  
  void (*callback)(void*) = NULL;
  void *context = NULL;
  
  if(mask_int & E1000_ICR_TXDW) {
    DEBUG("handle the txdw interrupt\n");
    e1000_unmap_callback(dev_state->tx_map, 
                         (uint64_t **)&callback, (void **)&context);
    if(callback && context) {     
      DEBUG("invoke callback function callback: 0x%p\n", callback);
      callback(context);
    } else {
      ERROR("no callback function!!!\n");
      return -1;
    }
  } else if(mask_int & E1000_ICR_RXT0) {
    DEBUG("handle the rxt0 interrupt\n");
    e1000_unmap_callback(dev_state->tx_map, 
                         (uint64_t **)&callback, (void **)&context);
    if(callback && context) {     
      DEBUG("invoke callback function callback: 0x%p\n", callback);
      callback(context);
    } else {
      ERROR("no callback function!!!\n");
      return -1;
    }    
  }
  // must have this line at the end of the handler
  IRQ_HANDLER_END();
  DEBUG("end irq\n\n\n");
  return 0;
}

static struct nk_net_dev_int ops={
  .get_characteristics = e1000_get_characteristics,
  .post_receive        = e1000_post_receive,
  .post_send           = e1000_post_send,
};

int e1000_pci_init(struct naut_info * naut)
{
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
        state->mac_addr = mac_all;
        DEBUG("e1000 mac_all = 0x%lX\n", mac_all);
        DEBUG("e1000 mac_high = 0x%x mac_low = 0x%x\n", mac_high, mac_low);
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
  // -> interrupt when the devive receives a package 
  WRITE_MEM(state->dev, E1000_RDTR_OFFSET, 0);
  // enable only transmit descriptor written back and
  // receive interrupt timer 
  WRITE_MEM(state->dev, E1000_IMS_OFFSET, E1000_ICR_TXDW | E1000_ICR_RXT0);
  
  uint32_t status = 0;
  status = READ_MEM(state->dev, E1000_IMS_OFFSET);
  DEBUG("IMS: %d\n", status);
  status = READ_MEM(state->dev, E1000_ICR_OFFSET);
  DEBUG("*** e1000 ICR = 0x%08x TXQE 0x%08x\n", status, E1000_ICR_TXQE & status);
  DEBUG("*** e1000 ICR = 0x%08x TXD_LOW 0x%08x\n", status, E1000_ICR_TXD_LOW & status);
  DEBUG("*** e1000 ICR = 0x%08x TXDW 0x%08x\n", status, E1000_ICR_TXDW & status);
  e1000_init_transmit_ring(TX_BLOCKSIZE, TX_DSC_COUNT, state);
  e1000_init_receive_ring(RX_BLOCKSIZE, RX_DSC_COUNT, state);
  // return 0;
  // ***************** INIT IS COMPLETE ************************* //
  DEBUG("total pkt tx=%d\n", READ_MEM(state->dev, E1000_TPT_OFFSET));
  DEBUG("total pkt rx=%d\n", READ_MEM(state->dev, E1000_TPR_OFFSET));
  e1000_send_packet(my_packet, (uint16_t)sizeof(my_packet), state, 0);
  e1000_send_packet(my_packet, (uint16_t)sizeof(my_packet), state, 0);

  uint64_t* my_rcv_space = malloc(8*1024);
  DEBUG("receiving a packet\n");
  e1000_receive_packet(my_rcv_space, 8*1024, state);
  /* DEBUG("rx tail_pos: %d RDT: %d\n", */
  /*       RXD_TAIL, READ_MEM(state->dev, RDT_OFFSET)); */
  e1000_receive_packet(my_rcv_space, 8*1024, state);
  /* DEBUG("rx tail_pos: %d RDT: %d\n", */
  /*       RXD_TAIL, READ_MEM(state->dev, RDT_OFFSET)); */
  /* status = READ_MEM(state->dev, E1000_ICR_OFFSET);   */
  /* DEBUG("*** e1000 ICR = 0x%08x SRPD 0x%08x\n", status, E1000_ICR_SRPD & status); */
  //e1000_receive_packet(my_rcv_space, 8*1024, state);
  dump_packet((uint8_t *) my_rcv_space, 128);
  DEBUG("total pkt tx=%d\n", READ(state->dev, E1000_TPT_OFFSET));
  DEBUG("total pkt rx=%d\n", READ(state->dev, E1000_TPR_OFFSET));
  return 0;
}

int e1000_pci_deinit()
{
  INFO("deinited\n");
  return 0;
}
