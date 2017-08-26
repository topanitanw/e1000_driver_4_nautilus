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

#include <nautilus/nautilus.h>
#include <nautilus/netdev.h>
#include <nautilus/cpu.h>
#include <dev/pci.h>
#include <nautilus/mm.h>              // malloc, free
#include <dev/e1000e_pci.h>
#include <nautilus/irq.h>             // interrupt register
#include <nautilus/naut_string.h>     // memset, memcpy
#include <nautilus/dev.h>             // NK_DEV_REQ_*
#include <nautilus/timer.h>           // nk_sleep(ns);
#include <nautilus/cpu.h>             // udelay

#ifndef NAUT_CONFIG_DEBUG_E1000E_PCI
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...)
#endif

#define INFO(fmt, args...)     INFO_PRINT("E1000E_PCI: " fmt, ##args)
#define DEBUG(fmt, args...)    DEBUG_PRINT("E1000E_PCI: " fmt, ##args)
#define ERROR(fmt, args...)    ERROR_PRINT("E1000E_PCI: " fmt, ##args)
#define READ_MEM(d, o)         (*((volatile uint32_t*)(((d)->mem_start)+(o))))
#define WRITE_MEM(d, o, v)     ((*((volatile uint32_t*)(((d)->mem_start)+(o))))=(v))

#define RXD_RING (state->rxd_ring) // e1000e_ring *
#define RXD_PREV_HEAD (RXD_RING->head_prev)
#define RXD_TAIL (RXD_RING->tail_pos)
#define RX_PACKET_BUFFER (RXD_RING->packet_buffer) // void *, packet buff addr
#define RXD_RING_BUFFER (RXD_RING->ring_buffer) // void *, ring buff addr
#define RX_DESC(i) (((struct e1000e_rx_desc*)RXD_RING_BUFFER)[(i)])
#define RXD_ERRORS(i) (((struct e1000e_rx_desc*)RXD_RING_BUFFER)[(i)].errors)
#define RXD_STATUS(i) (((struct e1000e_rx_desc*)RXD_RING_BUFFER)[(i)].status)
#define RXD_LENGTH(i) (((struct e1000e_rx_desc*)RXD_RING_BUFFER)[(i)].length)
#define RXD_ADDR(i) (((struct e1000e_rx_desc*)RXD_RING_BUFFER)[(i)].addr)

#define RXD_COUNT (RXD_RING->count)
#define RXD_INC(a,b) (((a) + (b)) % RXD_RING->count) // modular increment rxd index

#define TXD_RING (state->tx_ring) // e1000e_ring *
#define TXD_PREV_HEAD (TXD_RING->head_prev)
#define TXD_TAIL (TXD_RING->tail_pos)
#define TX_PACKET_BUFFER (TXD_RING->packet_buffer) // void *, packet buff addr
#define TXD_RING_BUFFER (TXD_RING->ring_buffer) // void *, ring buff addr
#define TXD_CMD(i) (((struct e1000e_tx_desc*)TXD_RING_BUFFER)[(i)].cmd)
#define TXD_STATUS(i) (((struct e1000e_tx_desc*)TXD_RING_BUFFER)[(i)].status)
#define TXD_LENGTH(i) (((struct e1000e_tx_desc*)TXD_RING_BUFFER)[(i)].length)
#define TXD_ADDR(i) (((struct e1000e_tx_desc*)TXD_RING_BUFFER)[(i)].addr)

#define TXD_COUNT (TXD_RING->count)
#define TXD_INC(a,b) (((a) + (b)) % TXD_RING->count) // modular increment txd index

// a is old index, b is incr amount, c is queue size
#define TXMAP (state->tx_map)
#define RXMAP (state->rx_map)

#define IRQ_NUMBER       11

static struct e1000e_state *dev_state = NULL;

inline uint64_t READ_MEM64(struct e1000e_dev* dev, uint64_t offset) {
  return *((volatile uint64_t*)(dev->mem_start +offset));
}

inline void WRITE_MEM64(struct e1000e_dev* dev, uint64_t offset, uint64_t val) {
  (*(volatile uint32_t*)(dev->mem_start+offset)) = val;
}

// initialize ring buffer to hold transmit descriptors
static int e1000e_init_transmit_ring(struct e1000e_state *state) {
  TXMAP = malloc(sizeof(struct e1000e_map_ring));
  if (!TXMAP) {
    ERROR("Cannot allocate txmap\n");
    return -1;
  }
  memset(TXMAP, 0, sizeof(struct e1000e_map_ring));
  TXMAP->ring_len = TX_DSC_COUNT;

  TXMAP->map_ring = malloc(sizeof(struct e1000e_fn_map) * TX_DSC_COUNT);
  if (!TXMAP->map_ring) {
    ERROR("Cannot allocate txmap->ring\n");
    return -1;
  }
  memset(TXMAP->map_ring, 0, sizeof(struct e1000e_fn_map) * TX_DSC_COUNT);

  TXD_RING = malloc(sizeof(struct e1000e_desc_ring));
  if (!TXD_RING) {
    ERROR("Cannot allocate TXD_RING\n");
    return -1;
  }
  memset(TXD_RING, 0, sizeof(struct e1000e_desc_ring));

  // allocate TX_DESC_COUNT transmit descriptors in the ring buffer.
  TXD_RING_BUFFER = malloc(sizeof(struct e1000e_tx_desc)*TX_DSC_COUNT);
  if (!TXD_RING_BUFFER) {
    ERROR("Cannot allocate TXD_RING_BUFFER\n");
    return -1;
  }
  memset(TXD_RING_BUFFER, 0, sizeof(struct e1000e_tx_desc)*TX_DSC_COUNT);
  TXD_COUNT = TX_DSC_COUNT;

  DEBUG("TX RING BUFFER at 0x%p\n", TXD_RING_BUFFER);
  // store the address of the memory in TDBAL/TDBAH
  WRITE_MEM(state->dev, E1000E_TDBAL_OFFSET,
            (uint32_t)( 0x00000000ffffffff & (uint64_t) TXD_RING_BUFFER));
  WRITE_MEM(state->dev, E1000E_TDBAH_OFFSET,
            (uint32_t)((0xffffffff00000000 & (uint64_t) TXD_RING_BUFFER) >> 32));
  DEBUG("TXD_RING_BUFFER=0x%016lx, TDBAH=0x%08x, TDBAL=0x%08x\n",
        TXD_RING_BUFFER, READ_MEM(state->dev, E1000E_TDBAH_OFFSET),
        READ_MEM(state->dev, E1000E_TDBAL_OFFSET));
  // write tdlen: transmit descriptor length
  WRITE_MEM(state->dev, E1000E_TDLEN_OFFSET,
            sizeof(struct e1000e_tx_desc) * TX_DSC_COUNT);
  // write the tdh, tdt with 0
  WRITE_MEM(state->dev, E1000E_TDT_OFFSET, 0);
  WRITE_MEM(state->dev, E1000E_TDH_OFFSET, 0);
  TXD_PREV_HEAD = 0;
  TXD_TAIL = 0;
  DEBUG("init tx fn: TDLEN = 0x%08x, TDH = 0x%08x, TDT = 0x%08x\n",
        READ_MEM(state->dev, E1000E_TDLEN_OFFSET),
        READ_MEM(state->dev, E1000E_TDH_OFFSET),
        READ_MEM(state->dev, E1000E_TDT_OFFSET));
  // write tctl register
  WRITE_MEM(state->dev, E1000E_TCTL_OFFSET,
            E1000E_TCTL_EN | E1000E_TCTL_PSP | E1000E_TCTL_COLD_FD);
  DEBUG("init tx fn: TCTL = 0x%08x expects 0x%08x\n",
        READ_MEM(state->dev, E1000E_TCTL_OFFSET),
        E1000E_TCTL_EN | E1000E_TCTL_PSP | E1000E_TCTL_COLD_FD);

  // write tipg regsiter
  // 00,00 0000 0110,0000 0010 00,00 0000 1010 = 0x0060200a
  // will be zero when emulating hardware
  WRITE_MEM(state->dev, E1000E_TIPG_OFFSET,
            E1000E_TIPG_IPGT | E1000E_TIPG_IPGR1_IEEE8023 | E1000E_TIPG_IPGR2_IEEE8023);

  DEBUG("init tx fn: TIPG = 0x%08x expects 0x%08x\n",
        READ_MEM(state->dev, E1000E_TIPG_OFFSET),
        E1000E_TIPG_IPGT | E1000E_TIPG_IPGR1_IEEE8023 | E1000E_TIPG_IPGR2_IEEE8023);
  return 0;
}

// initialize a ring buffer to hold receive descriptor
static int e1000e_init_receive_ring(struct e1000e_state *state) {
  RXMAP = malloc(sizeof(struct e1000e_map_ring));
  if (!RXMAP) {
    ERROR("Cannot allocate rxmap\n");
    return -1;
  }
  memset(RXMAP, 0, sizeof(struct e1000e_map_ring));

  RXMAP->map_ring = malloc(sizeof(struct e1000e_fn_map) * RX_DSC_COUNT);
  if (!RXMAP->map_ring) {
    ERROR("Cannot allocate rxmap->ring\n");
    return -1;
  }
  memset(RXMAP->map_ring, 0, sizeof(struct e1000e_fn_map) * RX_DSC_COUNT);
  RXMAP->ring_len = RX_DSC_COUNT;

  RXD_RING = malloc(sizeof(struct e1000e_desc_ring));
  if (!RXD_RING) {
    ERROR("Cannot allocate rxd_ring buffer\n");
    return -1;
  }
  memset(RXD_RING, 0, sizeof(struct e1000e_desc_ring));

  // the number of the receive descriptor in the ring
  RXD_COUNT = RX_DSC_COUNT;

  // allocate the receive descriptor ring buffer
  uint32_t rx_desc_size = sizeof(struct e1000e_rx_desc) * RX_DSC_COUNT;
  RXD_RING_BUFFER = malloc(rx_desc_size);
  if (!RXD_RING_BUFFER) {
    ERROR("Cannot allocate RXD_RING_BUFFER\n");
    return -1;
  }
  memset(RXD_RING_BUFFER, 0, rx_desc_size);

  // store the address of the memory in TDBAL/TDBAH
  WRITE_MEM(state->dev, E1000E_RDBAL_OFFSET,
            (uint32_t)(0x00000000ffffffff & (uint64_t) RXD_RING_BUFFER));
  WRITE_MEM(state->dev, E1000E_RDBAH_OFFSET,
            (uint32_t)((0xffffffff00000000 & (uint64_t) RXD_RING_BUFFER) >> 32));
  DEBUG("init rx fn: RDBAH = 0x%08x, RDBAL = 0x%08x = rd_buffer\n",
        READ_MEM(state->dev, E1000E_RDBAH_OFFSET),
        READ_MEM(state->dev, E1000E_RDBAL_OFFSET));
  DEBUG("init rx fn: rd_buffer = 0x%016lx\n", RXD_RING_BUFFER);

  // write rdlen
  WRITE_MEM(state->dev, E1000E_RDLEN_OFFSET, rx_desc_size);
  DEBUG("init rx fn: RDLEN=0x%08x should be 0x%08x\n",
        READ_MEM(state->dev, E1000E_RDLEN_OFFSET), rx_desc_size);
  // write the rdh, rdt with 0
  WRITE_MEM(state->dev, E1000E_RDH_OFFSET, 0);
  WRITE_MEM(state->dev, E1000E_RDT_OFFSET, 0);
  DEBUG("init rx fn: RDH=0x%08x, RDT=0x%08x expects 0\n",
        READ_MEM(state->dev, E1000E_RDH_OFFSET),
        READ_MEM(state->dev, E1000E_RDT_OFFSET));
  RXD_PREV_HEAD = 0;
  RXD_TAIL = 0;
  // write rctl register specifing the receive mode
  uint32_t rctl_reg = E1000E_RCTL_EN | E1000E_RCTL_SBP | E1000E_RCTL_UPE | E1000E_RCTL_LPE | E1000E_RCTL_DTYP_LEGACY | E1000E_RCTL_BAM | E1000E_RCTL_PMCF | E1000E_RCTL_RDMTS_HALF | E1000E_RCTL_BSIZE_2048;

  // receive buffer threshold and size
  DEBUG("init rx fn: rctl_reg = 0x%08x, expected value: 0x%08x\n",
        rctl_reg, 0x0083832e);
  // WRITE_MEM(state->dev, RCTL_OFFSET, 0x0083832e);
  WRITE_MEM(state->dev, E1000E_RCTL_OFFSET, rctl_reg);
  DEBUG("init rx fn: RCTL=0x%08x expects  0x%08x\n",
        READ_MEM(state->dev, E1000E_RCTL_OFFSET),
        rctl_reg);
  return 0;
}

static int e1000e_send_packet(uint8_t* packet_addr,
                              uint64_t packet_size,
                              struct e1000e_state *state) {
  DEBUG("e1000e_send_packet fn\n");
  DEBUG("packet_addr 0x%p packet_size: %d\n", packet_addr, packet_size);
  TXD_TAIL = READ_MEM(state->dev, E1000E_TDT_OFFSET);
  DEBUG("status before sending a packet: TDH = %d TDT = %d tail_pos = %d\n",
        READ_MEM(state->dev, E1000E_TDH_OFFSET),
        READ_MEM(state->dev, E1000E_TDT_OFFSET),
        TXD_TAIL);
  DEBUG("tpt total packet transmit: %d\n", READ_MEM(state->dev, E1000E_TPT_OFFSET));
  if(packet_size > MAX_TU) {
    ERROR("packet is too large.\n");
    return -1;
  }

  // e1000e_init_single_txd(TXD_TAIL, state); // make new descriptor
  memset(((struct e1000e_tx_desc *)TXD_RING_BUFFER + TXD_TAIL),
         0, sizeof(struct e1000e_tx_desc));
  TXD_ADDR(TXD_TAIL) = (uint64_t*) packet_addr;
  TXD_LENGTH(TXD_TAIL) = packet_size;
  // set up send flags
  TXD_CMD(TXD_TAIL).dext = 0;
  TXD_CMD(TXD_TAIL).vle = 0;
  TXD_CMD(TXD_TAIL).ifcs = 1;
  // set the end of packet flag if this is the last fragment
  TXD_CMD(TXD_TAIL).eop = 1;
  // interrupt delay enable
  // if ide = 0 and rs = 1, the transmit interrupt will occur immediately
  // after the packet is sent.
  // TXD_CMD(TXD_TAIL).ide = 1;
  // report the status of the descriptor
  TXD_CMD(TXD_TAIL).rs = 1;

  // increment transmit descriptor list tail by 1
  DEBUG("moving the tail\n");
  TXD_TAIL = TXD_INC(1, TXD_TAIL);
  WRITE_MEM(state->dev, E1000E_TDT_OFFSET, TXD_TAIL);
  DEBUG("status after moving tail: TDH = %d TDT = %d tail_pos = %d\n",
        READ_MEM(state->dev, E1000E_TDH_OFFSET),
        READ_MEM(state->dev, E1000E_TDT_OFFSET),
        TXD_TAIL);
  DEBUG("transmit error: %d\n",
        TXD_STATUS(TXD_PREV_HEAD).ec | TXD_STATUS(TXD_PREV_HEAD).lc);
  DEBUG("tpt total packet transmit: %d\n", READ_MEM(state->dev, E1000E_TPT_OFFSET));
  DEBUG("e1000e_send_packet fn end\n");
  return 0;
}

static int e1000e_receive_packet(uint8_t* buffer,
                                 uint64_t buffer_size,
                                 struct e1000e_state *state) {
  DEBUG("e1000e receive packet fn buffer = 0x%p, len = %lu\n",
        buffer, buffer_size);
  DEBUG("before moving tail head: %d, tail: %d\n",
        READ_MEM(state->dev, E1000E_RDH_OFFSET),
        READ_MEM(state->dev, E1000E_RDT_OFFSET));
  // if the buffer size is changed,
  // let the network adapter know the new buffer size
  if(state->rx_buffer_size != buffer_size) {
    uint32_t rctl = READ_MEM(state->dev, E1000E_RCTL_OFFSET) & E1000E_RCTL_BSIZE_MASK;
    switch(buffer_size) {
      case E1000E_RECV_BSIZE_256:
        rctl |= E1000E_RCTL_BSIZE_256;
        break;
      case E1000E_RECV_BSIZE_512:
        rctl |= E1000E_RCTL_BSIZE_512;
        break;
      case E1000E_RECV_BSIZE_1024:
        rctl |= E1000E_RCTL_BSIZE_1024;
        break;
      case E1000E_RECV_BSIZE_2048:
        rctl |= E1000E_RCTL_BSIZE_2048;
        break;
      case E1000E_RECV_BSIZE_4096:
        rctl |= E1000E_RCTL_BSIZE_4096;
        break;
      case E1000E_RECV_BSIZE_8192:
        rctl |= E1000E_RCTL_BSIZE_8192;
        break;
      case E1000E_RECV_BSIZE_16384:
        rctl |= E1000E_RCTL_BSIZE_16384;
        break;
      default:
        ERROR("Unmatch buffer size\n");
        return -1;
    }
    WRITE_MEM(state->dev, E1000E_RCTL_OFFSET, rctl);
    state->rx_buffer_size = buffer_size;
  }

  RXD_PREV_HEAD = READ_MEM(state->dev, E1000E_RDH_OFFSET);
  RXD_TAIL = READ_MEM(state->dev, E1000E_RDT_OFFSET);
  memset(((struct e1000e_rx_desc *)RXD_RING_BUFFER + TXD_TAIL),
         0, sizeof(struct e1000e_rx_desc));
  RXD_ADDR(RXD_TAIL) = (uint64_t*) buffer;
  WRITE_MEM(state->dev, E1000E_RDT_OFFSET, RXD_INC(RXD_TAIL, 1));
  RXD_TAIL = RXD_INC(RXD_TAIL, 1);
  DEBUG("after moving tail head: %d, prev_head: %d tail: %d\n",
        READ_MEM(state->dev, E1000E_RDH_OFFSET), RXD_PREV_HEAD,
        READ_MEM(state->dev, E1000E_RDT_OFFSET));

  DEBUG("end e1000e receive paket fn -----------------------\n");
  return 0;
}

uint64_t e1000e_packet_size_to_buffer_size(uint64_t sz) {
  // Round up the number to the buffer size with power of two
  // In E1000E, the packet buffer is the power of two.
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
  if(sz >= E1000E_RECV_BSIZE_MAX) {
    return E1000E_RECV_BSIZE_MAX;
  } else {
    return sz;
  }
}

int e1000e_get_characteristics(void *vstate, struct nk_net_dev_characteristics *c) {
  if (!vstate) {
    ERROR("The device state pointer is null\n");
    return -1;
  }

  if(!c) {
    ERROR("The characteristics pointer is null\n");
    return -1;
  }

  struct e1000e_state *state=(struct e1000e_state*)vstate;
  memcpy(c->mac, (void *) state->mac_addr, MAC_LEN);
  // minimum and the maximum transmission unit
  c->min_tu = MIN_TU;
  c->max_tu = MAX_TU;
  c->packet_size_to_buffer_size = e1000e_packet_size_to_buffer_size;
  return 0;
}

static struct nk_net_dev_int ops = {
  .get_characteristics = e1000e_get_characteristics,
  .post_receive        = e1000e_post_receive,
  .post_send           = e1000e_post_send,
};

static int e1000e_unmap_callback(struct e1000e_map_ring* map,
                                 uint64_t** callback,
                                 void** context) {
  // callback is a function pointer
  DEBUG("unmap callback fn head_pos %d tail_pos %d\n",
        map->head_pos, map->tail_pos);
  if(map->head_pos == map->tail_pos) {
    // if there is an empty mapping ring buffer, do not unmap the callback
    ERROR("Try to unmap an empty queue\n");
    return -1;
  }

  uint64_t i = map->head_pos;
  // TODO(panitan)
  *callback = (uint64_t *) map->map_ring[i].callback;
  *context =  map->map_ring[i].context;
  map->map_ring[i].callback = NULL;
  map->map_ring[i].context = NULL;
  map->head_pos = (1 + map->head_pos) % map->ring_len;
  DEBUG("end unmap callback fn head_pos %d tail_pos %d\n",
        map->head_pos, map->tail_pos);
  return 0;
}

static int e1000e_map_callback(struct e1000e_map_ring* map,
                               void (*callback)(nk_net_dev_status_t, void*),
                               void* context) {
  DEBUG("map callback head_pos %d tail_pos %d\n", map->head_pos, map->tail_pos);
  if(map->head_pos == ((map->tail_pos + 1) % map->ring_len)) {
    // when the mapping callback queue is full
    ERROR("Callback mapping queue is full.\n");
    return -1;
  }

  uint64_t i = map->tail_pos;
  struct e1000e_fn_map *fnmap = (map->map_ring + i);
  fnmap->callback = callback;
  fnmap->context = (uint64_t *)context;
  map->tail_pos = (1 + map->tail_pos) % map->ring_len;
  DEBUG("mapped callback head_pos: %d, tail_pos: %d\n", map->head_pos, map->tail_pos);
  return 0;
}

int e1000e_post_send(void *state,
                     uint8_t *src,
                     uint64_t len,
                     void (*callback)(nk_net_dev_status_t, void *),
                     void *context) {
  // always map callback
  int result = 0;
  DEBUG("post tx fn: callback 0x%p\n", callback);
  result = e1000e_map_callback(((struct e1000e_state*)state)->tx_map, callback, context);

  if (!result)
    result = e1000e_send_packet(src, len, (struct e1000e_state*) state);
  DEBUG("post tx fn: end\n");
  return result;
}

int e1000e_post_receive(void *vstate,
                        uint8_t *src,
                        uint64_t len,
                        void (*callback)(nk_net_dev_status_t, void *),
                        void *context) {
  // mapping the callback always
  // if result != -1 receive packet
  struct e1000e_state* state = (struct e1000e_state*) vstate;
  int result = 0;
  DEBUG("post rx fn: callback 0x%p\n", callback);
  result  = e1000e_map_callback(((struct e1000e_state*)state)->rx_map, callback, context);

  if(!result)
    result = e1000e_receive_packet(src, len, state);
  DEBUG("post rx fn: end\n");

  uint32_t status_pci = pci_cfg_readw(state->bus_num, state->dev_num,
                                      0, E1000E_PCI_STATUS_OFFSET);

  DEBUG("post rx fn: status_pci 0x%04x int %d\n",
        status_pci, status_pci & E1000E_PCI_STATUS_INT);
  return result;
}

static int e1000e_irq_handler(excp_entry_t * excp, excp_vec_t vec, void *s) {
  panic("within irq handler\n");
  DEBUG("irq_handler fn: vector: 0x%x rip: 0x%p\n", vec, excp->rip);
  struct e1000e_state* state = s;  
  uint32_t icr = READ_MEM(state->dev, E1000E_ICR_OFFSET);
  uint32_t ims = READ_MEM(state->dev, E1000E_IMS_OFFSET);
  uint32_t mask_int = icr & ims;
  DEBUG("irq_handler fn: ICR: 0x%08x IMS: 0x%08x mask_int: 0x%08x\n",
        icr, ims, mask_int);
  DEBUG("irq_handler fn: ICR: 0x%08x icr expects  zero.\n",
        READ_MEM(state->dev, E1000E_ICR_OFFSET));

  void (*callback)(nk_net_dev_status_t, void*) = NULL;
  void *context = NULL;
  nk_net_dev_status_t status = NK_NET_DEV_STATUS_SUCCESS;

  if(mask_int & E1000E_ICR_TXDW) {
    // transmit interrupt
    DEBUG("irq_handler fn: handle the txdw interrupt\n");
    e1000e_unmap_callback(state->tx_map, (uint64_t **)&callback, (void **)&context);
    // if there is an error while sending a packet, set the error status
    if(TXD_STATUS(TXD_PREV_HEAD).ec || TXD_STATUS(TXD_PREV_HEAD).lc) {
      ERROR("irq_handler fn: transmit errors\n");
      status = NK_NET_DEV_STATUS_ERROR;
    }

    // update the head of the ring buffer
    TXD_PREV_HEAD = TXD_INC(1, TXD_PREV_HEAD);
    DEBUG("irq_handler fn: total packet transmitted = %d\n",
          READ_MEM(state->dev, E1000E_TPT_OFFSET));
  }

  if(mask_int & E1000E_ICR_RXT0) {
    // receive interrupt
    DEBUG("irq_handler fn: handle the rxt0 interrupt\n");
    e1000e_unmap_callback(state->rx_map, (uint64_t **)&callback, (void **)&context);
    // checking errors
    if(RXD_ERRORS(RXD_PREV_HEAD)) {
      ERROR("irq_handler fn: receive an error packet\n");
      status = NK_NET_DEV_STATUS_ERROR;
    }

    // in the irq, update only the head of the buffer
    RXD_PREV_HEAD = RXD_INC(1, RXD_PREV_HEAD);
    DEBUG("irq_handler fn: total packet received = %d\n",
          READ_MEM(state->dev, E1000E_TPR_OFFSET));
  }

  if(callback) {
    DEBUG("irq_handler fn: invoke callback function callback: 0x%p\n", callback);
    callback(status, context);
  }

  DEBUG("irq_handler fn: end irq\n\n\n");
  // must have this line at the end of the handler
  IRQ_HANDLER_END();
  return 0;
}

void e1000e_disable_all_int() {
  WRITE_MEM(dev_state->dev, E1000E_IMC_OFFSET, 0xffffffff);
  return;
}

void e1000e_trigger_int() {
  // imc is a written only register.
  WRITE_MEM(dev_state->dev, E1000E_IMC_OFFSET, 0xffffffff);
  // enable only transmit descriptor written back and receive interrupt timer  
  WRITE_MEM(dev_state->dev, E1000E_IMS_OFFSET, E1000E_ICR_TXDW | E1000E_ICR_RXT0);

  if ((E1000E_ICR_TXDW | E1000E_ICR_RXT0) != READ_MEM(dev_state->dev, E1000E_IMS_OFFSET)) {
    ERROR("IMS reg: the written and read values do not match\n");
    return;
  }
  // after the interrupt is turned on, the interrupt handler is called
  // due to the transmit descriptor queue empty.
  // set an interrupt on ICS register
  // ICS is w reg which means its value cannot be read back.
  WRITE_MEM(dev_state->dev, E1000E_ICS_OFFSET, E1000E_ICR_TXDW | E1000E_ICR_RXT0);
  uint32_t status_pci = pci_cfg_readw(dev_state->bus_num, dev_state->dev_num,
                                      0, E1000E_PCI_STATUS_OFFSET);
  DEBUG("trigger int fn: status_pci 0x%04x int %d\n",
        status_pci, status_pci & E1000E_PCI_STATUS_INT);  
  return;
}

void e1000e_trigger_int_num(uint32_t int_num) {
  // imc is a written only register.
  WRITE_MEM(dev_state->dev, E1000E_IMC_OFFSET, 0xffffffff);
  // enable only transmit descriptor written back and receive interrupt timer  
  WRITE_MEM(dev_state->dev, E1000E_IMS_OFFSET, int_num);
  if (int_num != READ_MEM(dev_state->dev, E1000E_IMS_OFFSET)) {
    ERROR("IMS reg: the written and read values do not match\n");
    return;
  }  
  // after the interrupt is turned on, the interrupt handler is called
  // due to the transmit descriptor queue empty.
  // set an interrupt on ICS register
  WRITE_MEM(dev_state->dev, E1000E_ICS_OFFSET, int_num);

  uint32_t status_pci = pci_cfg_readw(dev_state->bus_num, dev_state->dev_num,
                                      0, E1000E_PCI_STATUS_OFFSET);
  DEBUG("trigger int fn: status_pci 0x%04x int %d\n",
        status_pci, status_pci & E1000E_PCI_STATUS_INT);  
  return;
}

void e1000e_legacy_int_off() {
  uint16_t pci_cmd = E1000E_PCI_CMD_MEM_ACCESS_EN | E1000E_PCI_CMD_IO_ACCESS_EN | E1000E_PCI_CMD_LANRW_EN | E1000E_PCI_CMD_INT_DISABLE;
  DEBUG("legacy int off: pci cmd: 0x%04x\n", pci_cmd);
  pci_cfg_writew(dev_state->bus_num,dev_state->dev_num, 0, E1000E_PCI_CMD_OFFSET, pci_cmd);
  DEBUG("legacy int off: pci_cmd 0x%04x expects  0x%04x\n",
        pci_cfg_readw(dev_state->bus_num, dev_state->dev_num, 0, E1000E_PCI_CMD_OFFSET),
        pci_cmd);
  DEBUG("legacy int off: pci status 0x%04x\n",
        pci_cfg_readw(dev_state->bus_num,dev_state->dev_num, 0, E1000E_PCI_STATUS_OFFSET));
  return;
}

void e1000e_legacy_int_on() {
  uint16_t pci_cmd = E1000E_PCI_CMD_MEM_ACCESS_EN | E1000E_PCI_CMD_IO_ACCESS_EN | E1000E_PCI_CMD_LANRW_EN;
  DEBUG("legacy int on: pci cmd: 0x%04x\n", pci_cmd);
  pci_cfg_writew(dev_state->bus_num,dev_state->dev_num, 0, E1000E_PCI_CMD_OFFSET, pci_cmd);
  DEBUG("legacy int on: pci_cmd 0x%04x expects  0x%04x\n",
        pci_cfg_readw(dev_state->bus_num, dev_state->dev_num, 0, E1000E_PCI_CMD_OFFSET),
        pci_cmd);
  DEBUG("legacy int on: pci status 0x%04x\n",
        pci_cfg_readw(dev_state->bus_num,dev_state->dev_num, 0, E1000E_PCI_STATUS_OFFSET));
  return;
}

int e1000e_pci_init(struct naut_info * naut) {
  // !!!
  struct e1000e_state *state = malloc(sizeof(struct e1000e_state));
  dev_state = state;
  struct pci_info *pci = naut->sys.pci;
  struct list_head *curbus, *curdev;
  uint16_t num = 0;

  INFO("e1000e pci init fn\n");
  // building the e1000e linked list and register it
  // linked list of e1000e devices
  struct list_head dev_list;
  INIT_LIST_HEAD(&dev_list);

  if (!pci) {
    ERROR("No PCI info\n");
    return -1;
  }

  DEBUG("Finding e1000e devices\n");

  list_for_each(curbus,&(pci->bus_list)) {
    struct pci_bus *bus = list_entry(curbus,struct pci_bus,bus_node);

    DEBUG("Searching PCI bus %u for E1000E devices\n", bus->num);

    list_for_each(curdev, &(bus->dev_list)) {
      struct pci_dev *pdev = list_entry(curdev,struct pci_dev,dev_node);
      struct pci_cfg_space *cfg = &pdev->cfg;

      DEBUG("Device %u is a 0x%x:0x%x\n", pdev->num, cfg->vendor_id, cfg->device_id);
      // intel vendor id and e1000e device id
      if (cfg->vendor_id==INTEL_VENDOR_ID && cfg->device_id==E1000E_DEVICE_ID) {
        DEBUG("Found E1000E Device\n");
        struct e1000e_dev *dev = malloc(sizeof(struct e1000e_dev));
        if (!dev) {
          ERROR("Cannot allocate e1000e_dev variable\n");
          return -1;
        }

        memset(dev,0,sizeof(*dev));
        state->dev = dev;
        dev->pci_dev = pdev;

        // PCI Interrupt (A..D)
        dev->pci_intr = cfg->dev_cfg.intr_pin;
        // Figure out mapping here or look at capabilities for MSI-X
        // dev->intr_vec = ... (for future work)

        // find out the bar for e1000e
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

          uint32_t start = 0;
          if (bar & 0x1) {
            start = dev->ioport_start = bar & 0xffffffc0;
            dev->ioport_end = dev->ioport_start + size;
          }

          if (!(bar & 0xe) && (i == 0)) {
            start = dev->mem_start = bar & 0xfffffff0;
            dev->mem_end = dev->mem_start + size;
          }

          DEBUG("bar %d is %s address=0x%x size=0x%x\n", i,
                bar & 0x1 ? "io port":"memory", start, size);
        }

        INFO("Adding e1000e device: bus=%u dev=%u func=%u: pci_intr=%u intr_vec=%u ioport_start=%p ioport_end=%p mem_start=%p mem_end=%p\n",
             bus->num, pdev->num, 0,
             dev->pci_intr, dev->intr_vec,
             dev->ioport_start, dev->ioport_end,
             dev->mem_start, dev->mem_end);
        // CLEANUP
        /* uint16_t old_cmd = pci_cfg_readw(bus->num,pdev->num,0,E1000E_PCI_CMD_OFFSET); */
        /* DEBUG("Old PCI CMD: 0x%04x\n",old_cmd); */

        /* old_cmd |= 0x7;  // make sure bus master is enabled */
        /* old_cmd &= ~0x40; */
        uint16_t pci_cmd = E1000E_PCI_CMD_MEM_ACCESS_EN | E1000E_PCI_CMD_IO_ACCESS_EN | E1000E_PCI_CMD_LANRW_EN | E1000E_PCI_CMD_INT_DISABLE;
        DEBUG("New PCI CMD: 0x%04x\n", pci_cmd);
        pci_cfg_writew(bus->num,pdev->num,0,E1000E_PCI_CMD_OFFSET, pci_cmd);
        DEBUG("init fn: pci_cmd 0x%04x expects  0x%04x\n",
              pci_cfg_readw(bus->num,pdev->num, 0, E1000E_PCI_CMD_OFFSET),
              pci_cmd);
        DEBUG("init fn: pci status 0x%04x\n",
              pci_cfg_readw(bus->num,pdev->num, 0, E1000E_PCI_STATUS_OFFSET));
        list_add(&dev_list, &dev->e1000e_node);
        sprintf(state->name, "e1000e-%d", num);
        num++;
        state->bus_num = bus->num;
        state->dev_num = pdev->num;
        dev->netdev = nk_net_dev_register(state->name, 0, &ops, (void *)state);
        if(!dev->netdev) {
          ERROR("Cannot register the e1000e device \"%s\"", state->name);
          return -1;
        }
      }
    }
  }

  if(!state->dev) {
    free(state);
    state = NULL;
    ERROR("init fn: Cannot find the e1000e device");
    return -1;
  }

  // disable interrupts
  WRITE_MEM(state->dev, E1000E_IMC_OFFSET, 0xffffffff);
  DEBUG("device reset\n");
  WRITE_MEM(state->dev, E1000E_CTRL_OFFSET, E1000E_CTRL_RST);
  udelay(RESTART_DELAY); // delay about 10 ms approximately
  // disable interrupts again
  WRITE_MEM(state->dev, E1000E_IMC_OFFSET, 0xffffffff);

  // WRITE_MEM(state->dev, E1000E_RAL_OFFSET, 0x05050505);
  uint32_t mac_low = READ_MEM(state->dev, E1000E_RAL_OFFSET);
  uint32_t mac_high = READ_MEM(state->dev, E1000E_RAH_OFFSET);
  // WRITE_MEM(state->dev, E1000E_RAH_OFFSET, (mac_high & 0xffff0000) | 0x0404);
  mac_high = READ_MEM(state->dev, E1000E_RAH_OFFSET);
  uint64_t mac_all = ((uint64_t)mac_low+((uint64_t)mac_high<<32)) & 0xffffffffffff;
  DEBUG("e1000e mac_all = 0x%lX\n", mac_all);
  DEBUG("e1000e mac_high = 0x%x mac_low = 0x%x\n", mac_high, mac_low);
  // set the bit 22th of GCR register
  uint32_t gcr_old = READ_MEM(state->dev, E1000E_GCR_OFFSET);
  WRITE_MEM(state->dev, E1000E_GCR_OFFSET, gcr_old | E1000E_GCR_B22);
  uint32_t gcr_new = READ_MEM(state->dev, E1000E_GCR_OFFSET);
  DEBUG("init fn: GCR = 0x%08x old gcr = 0x%08x tested %s\n",
        gcr_new, gcr_old,
        gcr_new & E1000E_GCR_B22 ? "pass":"fail");
  WRITE_MEM(state->dev, E1000E_GCR_OFFSET, E1000E_GCR_B22);

  uint32_t ctrl_reg = (E1000E_CTRL_FD | E1000E_CTRL_FRCSPD | E1000E_CTRL_FRCDPLX | E1000E_CTRL_SLU | E1000E_CTRL_SPEED_1G) & ~E1000E_CTRL_ILOS; // p50 manual
  WRITE_MEM(state->dev, E1000E_CTRL_OFFSET, ctrl_reg);
  WRITE_MEM(state->dev, E1000E_FCAL_OFFSET, 0);
  WRITE_MEM(state->dev, E1000E_FCAH_OFFSET, 0);
  WRITE_MEM(state->dev, E1000E_FCT_OFFSET, 0);
  DEBUG("init fn: e1000e ctrl = 0x%08x expects 0x%08x\n",
        READ_MEM(state->dev, E1000E_CTRL_OFFSET),
        ctrl_reg);
  uint32_t status_reg = READ_MEM(state->dev, E1000E_STATUS_OFFSET);
  DEBUG("init fn: e1000e status = 0x%08x\n", status_reg);
  DEBUG("init fn: does status.fd = 0x%01x? %s\n",
        E1000E_STATUS_FD, status_reg & E1000E_STATUS_FD ? "yes":"no");
  DEBUG("init fn: status.speed 0x%02x\n",
        (status_reg & E1000E_STATUS_SPEED_MASK) >> 8);
  DEBUG("init fn: status.lu 0x%01x %s\n",
        (status_reg & E1000E_STATUS_LU) >> 1,
        (status_reg & E1000E_STATUS_LU) ? "link is up.":"link is down.");

  e1000e_init_receive_ring(state);
  e1000e_init_transmit_ring(state);
  WRITE_MEM(state->dev, E1000E_IMC_OFFSET, 0);
  DEBUG("init fn: IMC = 0x%08x expects 0x%08x\n",
        READ_MEM(state->dev, E1000E_IMC_OFFSET), 0);
  struct pci_dev* pdev = pci_find_device(state->bus_num, state->dev_num, 0);
  if(pdev != NULL) {
    if (pci_dev_enable_msi(pdev, 153, 1, 0)) {
      // failed to enable...
      panic("cannot enable the msi\n");
      return 0;
    }
    
    if (register_int_handler(153, e1000e_irq_handler, NULL)) {
      // failed....
      panic("cannot register handler\n");
      return 0;
    }
    if (pci_dev_unmask_msi(pdev, 153)) {
      panic("cannot unmask msi\n");
      return 0;
    }
  } else {
    panic("pci_find_device returns null\n");
    return 0;
  }
  // register the interrupt handler
  // register_irq_handler(IRQ_NUMBER, e1000e_irq_handler, NULL);
  /* nk_unmask_irq(11); */
  /* for(int i = 12; i < 256; i++) { */
  /*   DEBUG("init fn: unmask irq %d\n", i); */
  /*   nk_unmask_irq(i); */
  /* } */
  /* nk_unmask_irq(IRQ_NUMBER); */
  // interrupt delay value = 0 -> does not delay
  WRITE_MEM(state->dev, E1000E_TIDV_OFFSET, 0);
  // receive interrupt delay timer = 0
  // -> interrupt when the device receives a package
  WRITE_MEM(state->dev, E1000E_RDTR_OFFSET, 0);
  // enable only transmit descriptor written back and receive interrupt timer
  // WRITE_MEM(state->dev, E1000E_IMS_OFFSET, E1000E_ICR_TXDW | E1000E_ICR_RXT0);
  // after the interrupt is turned on, the interrupt handler is called
  // due to the transmit descriptor queue empty.

  /* WRITE_MEM(state->dev, E1000E_ICS_OFFSET, E1000E_ICR_TXDW | E1000E_ICR_RXT0); */
  /* DEBUG("init fn: ICS 0x08x expect 0x08x", */
  /*       READ_MEM(state->dev, E1000E_ICS_OFFSET), */
  /*       E1000E_ICR_TXDW | E1000E_ICR_RXT0); */
  /* uint32_t status_pci = pci_cfg_readw(state->bus_num, state->dev_num, */
  /*                                     0, E1000E_PCI_STATUS_OFFSET); */
  /* DEBUG("init fn: status_pci 0x%04x int %d\n", */
  /*       status_pci, status_pci & E1000E_PCI_STATUS_INT); */

  uint32_t icr_reg = READ_MEM(state->dev, E1000E_ICR_OFFSET);
  DEBUG("init fn: IMS = 0x%08x ICR = 0x%08x\n",
        READ_MEM(state->dev, E1000E_IMS_OFFSET), icr_reg);
  DEBUG("init fn: TXQE = 0x%08x TXD_LOW = 0x%08x TXDW = 0x%08x\n",
        icr_reg,
        E1000E_ICR_TXQE & icr_reg,
        E1000E_ICR_TXD_LOW & icr_reg,
        E1000E_ICR_TXDW & icr_reg);
  return 0;
}

int e1000e_pci_deinit() {
  INFO("deinited\n");
  return 0;
}