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
 * Copyright (c) 2017, E1000 Netdev <Panitan.W@u.northwesttern.edu>
 * Copyright (c) 2017, The V3VEE Project  <http://www.v3vee.org> 
 *                     The Hobbes Project <http://xstack.sandia.gov/hobbes>
 * All rights reserved.
 *
 * Author: Panitan Wongse-ammat <Panitan.W@u.northwesttern.edu>
 * Marc Warrior
 * Galen Lansbury
 * Peter Dinda
 *
 * This is free software.  You are permitted to use,
 * redistribute, and modify it as specified in the file "LICENSE.txt".
 */
#include <nautilus/nautilus.h>
#include <nautilus/thread.h>    // nk_start_thread
#include <nautilus/cpu.h>       // warrior
#include <nautilus/naut_string.h>             // memset, memcpy
#include <nautilus/netdev.h>
#include <nautilus/printk.h>
#include <dev/pci.h>
#include <dev/e1000_pci.h>
#include <nautilus/arp.h>
// #include <arpa/inet.h>          // inet_aton, inet_ntoa, inet_ntop
// #include <stdlib.h>
// #include <stdio.h>

#define NAUT_CONFIG_DEBUG_ARP 1
#ifndef NAUT_CONFIG_DEBUG_ARP
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...)
#endif

#define INFO(fmt, args...) INFO_PRINT("ARP INFO: " fmt, ##args)
#define DEBUG(fmt, args...) DEBUG_PRINT("ARP DEBUG: " fmt, ##args)
#define ERROR(fmt, args...) ERROR_PRINT("ARP ERROR: " fmt, ##args)

const uint8_t ARP_BROADCAST_MAC[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

int compare_mac(uint8_t* mac1, uint8_t* mac2) {
  return ((mac1[0] == mac2[0]) && (mac1[1] == mac2[1]) && \
          (mac1[2] == mac2[2]) && (mac1[3] == mac2[3]) && \
          (mac1[4] == mac2[4]) && (mac1[5] == mac2[5]));
}

uint16_t htons(uint16_t v) {
  return (v >> 8) | (v << 8);
}

uint32_t htonl(uint32_t v) {
  return htons(v >> 16) | (htons((uint16_t) v) << 16);
}

// convert a string of an ip address to uint32_t
uint32_t ip_strtoint(char* str) {
  uint8_t a, b, c, d;
  sscanf(str, "%hhu.%hhu.%hhu.%hhu", &a, &b, &c, &d );
  uint32_t ip_addr = ( a << 24 ) | ( b << 16 ) | ( c << 8 ) | d;
  return ip_addr;
}

// convert uint32_t representation of an ip address to
// a dot-decimal representation 
void ip_inttostr(uint32_t ipv4, char *buf, int len) {
  // TODO(Panitan) use len
  uint8_t *ptr = (uint8_t*)&ipv4;
  // printf("%u, %u, %u, %u : ", ptr[3], ptr[2], ptr[1], ptr[0]);
  uint32_t n = sprintf(buf, "%hhu.%hhu.%hhu.%hhu", ptr[3], ptr[2], ptr[1], ptr[0]);
  // printf("n: %u %s\n", n, buf);
}

void create_arp_pkt(struct arp_packet *pkt, uint16_t opcode,
                    uint32_t sender_ip, uint8_t *sender_mac,
                    uint32_t target_ip, uint8_t *target_mac) {
  pkt->hw_type = htons(ARP_ETHERNET_HW_TYPE);
  pkt->pro_type = htons(ARP_IP_PRO_TYPE);
  pkt->hw_len = MAC_LEN;
  pkt->pro_len = IPV4_LEN;
  pkt->opcode = htons(opcode);
  pkt->sender_ip_addr = htonl(sender_ip);
  memcpy((void*)pkt->sender_hw_addr, sender_mac, MAC_LEN);
  pkt->target_ip_addr = htonl(target_ip);  
  memcpy((void*)pkt->target_hw_addr, target_mac, MAC_LEN);
}

void print_arp_packet(struct arp_packet* pkt) {
  DEBUG("arp packet ------------------------------\n");
  switch(pkt->hw_type) {
    case(ARP_ETHERNET_HW_TYPE):
      DEBUG("\t hw_type =%d : ETHERNET\n", pkt->hw_type);
      break;
    default:
      DEBUG("\t hw_type =%d : UNKNOWN\n", pkt->hw_type);
  }

  DEBUG("\t pro_type =%d ; IP: %d\n",  pkt->pro_type, ARP_IP_PRO_TYPE);
  DEBUG("\t hw_len =%d; pro_len =%d\n; IP: %d",  pkt->hw_len, pkt->pro_len);

  switch(pkt->opcode) {
    case(ARP_REQUEST_OPCODE):
      DEBUG("\t opcode =%d : REQUEST\n", pkt->opcode);
      break;
    case(ARP_REPLY_OPCODE):
      DEBUG("\t opcode =%d : REPLY\n", pkt->opcode);
      break;      
    default:
      DEBUG("\t opcode =%d : UNKNOWN\n", pkt->opcode);
  }
  
  DEBUG("\t sender_hw_addr: %u-%u-%u-%u-%u-%u\n",
        pkt->sender_hw_addr[0],pkt->sender_hw_addr[1],pkt->sender_hw_addr[2],
        pkt->sender_hw_addr[3],pkt->sender_hw_addr[4],pkt->sender_hw_addr[5]);
  char buf[20];
  memset(buf, 0, sizeof(buf));
  ip_inttostr(pkt->sender_ip_addr, buf, 20);
  DEBUG("\t sender_ip_addr: %d %s\n", pkt->sender_ip_addr, buf);
  DEBUG("\t target_hw_addr: %u-%u-%u-%u-%u-%u\n",
        pkt->target_hw_addr[0],pkt->target_hw_addr[1],pkt->target_hw_addr[2],
        pkt->target_hw_addr[3],pkt->target_hw_addr[4],pkt->target_hw_addr[5]);
  ip_inttostr(pkt->target_ip_addr, buf, 20);
  DEBUG("\t target_ip_addr: %d %s\n", pkt->target_ip_addr, buf);  
}

void arp_thread(void *in, void **out) {
  DEBUG("|arp_thread function ------------------------------|");
  struct arp_info * ai = (struct arp_info *)in;
  struct nk_net_dev_characteristics c;
  DEBUG("|get_characteristics function ------------------------------|");  
  nk_net_dev_get_characteristics(ai->netdev, &c);
  // now you have the MAC address in c
  // and the ip address in ai, so we have the binding once again

  uint8_t input_packet[c.max_tu];
  DEBUG("arp_thread before while ------------------------------");
  while (1) { 
    // wait to receive a packet - should check errors
    DEBUG("arp_thread while loop ------------------------------");
    nk_net_dev_receive_packet(ai->netdev, input_packet, c.max_tu, NK_DEV_REQ_BLOCKING);
    struct arp_packet *ip_pkt = (struct arp_packet*) input_packet;
    print_arp_packet(ip_pkt);
    // if(input packet is an ARP packet and its a request && it matches ai->ip_addr)
    if ((compare_mac((uint8_t *) ip_pkt->target_hw_addr,
                     (uint8_t *) ARP_BROADCAST_MAC) ||
         compare_mac((uint8_t *) ip_pkt->target_hw_addr, c.mac)) &&
        (ip_pkt->opcode == ARP_REQUEST_OPCODE) &&
        (ip_pkt->target_ip_addr == ai->ip_addr)) {    
      uint8_t output_packet[c.max_tu];
      // construct an ARP reply in output_packet
      struct arp_packet *op_pkt = (struct arp_packet*) output_packet;
      create_arp_pkt(op_pkt, ARP_REPLY_OPCODE,
                     ai->ip_addr, c.mac, 
                     (uint32_t) ip_pkt->sender_ip_addr,
                     (uint8_t *) ip_pkt->sender_hw_addr);
      print_arp_packet(op_pkt);
      nk_net_dev_send_packet(ai->netdev, output_packet,
                             sizeof(struct arp_packet),
                             NK_DEV_REQ_NONBLOCKING);
    }
  }
}

int arp_init(struct naut_info * naut)
{
  INFO("arp init ========================================\n");
  nk_thread_id_t tid;
  // e1000_pci_init(naut);
  // intiialize e1000, register it as, say, "e1000-0"
  // e1000_state->netdev = nk_netdev_register("e1000-0", 0, );
  // e1000_0_state.netdev = nk_net_dev_find("e1000-0");
  struct arp_info *ai = malloc(sizeof(*ai));
  if (!ai) {
    ERROR("Cannot allocate ai arp_info\n");
    return -1;
  }  
  //ai->netdev = e1000_0_state.netdev;   // store device to IP binding
  ai->netdev = nk_net_dev_find("e1000-0");
  DEBUG("ip_strtoint ========================================\n");
  ai->ip_addr = ip_strtoint(IP_ADDRESS_STRING);
  char buf[20];
  memset(buf,0, 20);
  DEBUG("ip address string: %s, ip address uint32_t: 0x%08x, ip address string: \n",
        IP_ADDRESS_STRING, ai->ip_addr);//, ip_inttostr(ai->ip_addr, buf, 20));
  /* // keep the ip address in network byte order */
  DEBUG("nk_thread_start ========================================\n");  
  if (nk_thread_start(arp_thread, (void*)ai , NULL, 1, PAGE_SIZE_4KB, &tid, 1)) {
    free(ai);
    return -1;
  } else {
    return 0;
  }
  // return 0;
}

int arp_deinit()
{
  INFO("deinited\n");
  return 0;
}
