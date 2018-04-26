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

#ifndef __E1000E_PCI
#define __E1000E_PCI

struct e1000e_state {
  // FIX compared with the e1000_pci.c
  // delete dev
  // add all the fields in dev to this structure
  char name[DEV_NAME_LEN];
  uint8_t mac_addr[6];
  // the bus number of the devince on the pci bus
  uint32_t bus_num;       
  // the device number of the device on the pci bus
  struct pci_dev* pci_dev;
  struct e1000e_dev *dev;
  struct e1000e_desc_ring *tx_ring;
  struct e1000e_desc_ring *rxd_ring;
  // a circular queue mapping between callback function and tx descriptor
  struct e1000e_map_ring *tx_map;
  // a circular queue mapping between callback funtion and rx descriptor
  struct e1000e_map_ring *rx_map;
  // the size of receive buffers
  uint64_t rx_buffer_size;
  // interrupt mark set
  uint32_t ims_reg;
};

struct tsc {
  uint64_t start;
  uint64_t end;
};
typedef struct tsc tsc_t;

struct operation {
  // should be a postx, but I did last week.
  tsc_t postx_map;
  tsc_t xpkt;
  tsc_t irq_unmap;
  tsc_t irq_callback;
  tsc_t irq;
  tsc_t dev_wait;
};
typedef struct operation op_t;

struct iteration {
  op_t tx;
  op_t rx;
};
typedef struct iteration iteration_t;

#define GET_TSC(x)                       ((x)=rdtsc())
#define DIFF_TSC(r,s,e)                  ((r)=(e)-(s))

// function declaration
int e1000e_pci_init(struct naut_info * naut); 
int e1000e_pci_deinit();
int e1000e_post_send(void*, uint8_t*, uint64_t,
                    void (*)(nk_net_dev_status_t, void*), void*);
int e1000e_post_receive(void*, uint8_t*, uint64_t,
                       void (*)(nk_net_dev_status_t, void*), void*);
void e1000e_opt_shell();
void e1000e_no_opt_shell();

// // TODO: panitan cleanup -> delete this function 
void e1000e_disable_all_int();
void e1000e_trigger_int();
void e1000e_trigger_int_num(uint32_t int_num);
void e1000e_legacy_int_off();
void e1000e_legacy_int_on();
void e1000e_interpret_int_shell();
void e1000e_interpret_int(struct e1000e_state*, uint32_t);
void e1000e_interpret_ims(struct e1000e_state*);
void e1000e_interpret_icr(struct e1000e_state*);
void e1000e_read_stat_shell();
void e1000e_reset_all_int();
void e1000e_reset_rxo();
void e1000e_interpret_rxd(struct e1000e_state*);
void e1000e_interpret_rxd_shell();
void e1000e_enable_psp_shell();
void e1000e_disable_psp_shell();
void e1000e_enable_srpd_int_shell(uint32_t size);
void e1000e_disable_srpd_int_shell();
void e1000e_interpret_rctl_shell();
void e1000e_interpret_tctl_shell();

uint32_t e1000e_read_speed_bit(uint32_t reg, uint32_t mask, uint32_t shift);
char* e1000e_read_speed_char(uint32_t reg, uint32_t mask, uint32_t shift);
void e1000e_read_write();

#endif
