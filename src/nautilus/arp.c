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

uint64_t htonll(uint64_t v) {
  return htonl(v >> 32) | ((uint64_t) htonl((uint32_t) v) << 32);
}

uint16_t ntohs(uint16_t v) {
  return htons(v);
}

uint32_t ntohl(uint32_t v) {
  return htonl(v);
}

uint64_t ntohll(uint64_t v) {
  return htonll(v);
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
  if(len < 16) return;
  uint8_t *ptr = (uint8_t*)&ipv4;
  // printf("%u, %u, %u, %u : ", ptr[3], ptr[2], ptr[1], ptr[0]);
  sprintf(buf, "%hhu.%hhu.%hhu.%hhu", ptr[3], ptr[2], ptr[1], ptr[0]);
  // printf("n: %u %s\n", n, buf);
}

void create_arp_pkt(struct arp_packet *pkt, uint16_t opcode,
                    uint32_t sender_ip, uint8_t *sender_mac,
                    uint32_t target_ip, uint8_t *target_mac) {
  pkt->hw_type = htons(ARP_HW_TYPE_ETHERNET);
  pkt->pro_type = htons(ARP_PRO_TYPE_IPV4);
  pkt->hw_len = MAC_LEN;
  pkt->pro_len = IPV4_LEN;
  pkt->opcode = htons(opcode);
  pkt->sender_ip_addr = htonl(sender_ip);
  memcpy((void*)pkt->sender_hw_addr, sender_mac, MAC_LEN);
  pkt->target_ip_addr = htonl(target_ip);  
  memcpy((void*)pkt->target_hw_addr, target_mac, MAC_LEN);
}

void mac_inttostr(uint8_t *mac, char* buf, int len) {
  if(buf && len < 24)
    return;
  /* sprintf(buf, "%d", 5); */
  sprintf(buf, "%x:%x:%x_%x:%x:%x", mac[0],mac[1],mac[2], mac[3],mac[4],mac[5]);
}

char to_hex(uint8_t n)
{
  n &= 0xf;
  if (n<10) {
    return '0'+n;
  } else {
    return 'a'+(n-10);
  }
}

void dump_packet(uint8_t *p, int len)
{
  char buf[len*3+1];
  uint8_t byte;
  int i;

  for (i=0; i<len; i++) {
    byte = p[i];
    buf[3*i] = to_hex((byte>>4) & 0xf);
    buf[3*i+1] = to_hex((byte & 0xf));
    if((i+1)%8) 
      buf[3*i+2] = ',';
    else
      buf[3*i+2] = ' ';
  }
  buf[len*3] = 0;
    
  DEBUG("Packet content %d bytes: %s\n", len, buf);
}

void print_ethernet_packet(struct ethernet_header* pkt) {
  DEBUG("eth packet ------------------------------\n");
  char buf[25];
  memset(buf, 0, sizeof(buf));
  
  mac_inttostr(pkt->dst_addr, buf, sizeof(buf));
  DEBUG("\t dst_addr: %s\n",buf);

  mac_inttostr(pkt->src_addr, buf, sizeof(buf));
  DEBUG("\t src_addr: %s\n",buf);
  
  uint16_t host16 = ntohs(pkt->hw_type);
  switch(host16) {
    case(ETHERNET_TYPE_ARP):
      DEBUG("\t type = 0x%x : ARP\n", host16);
      break;
    case(ETHERNET_TYPE_IPV4):
      DEBUG("\t type = 0x%x : IPv4\n", host16);
      break;
    case(ETHERNET_TYPE_IPX):
      DEBUG("\t type = 0x%x : IPx\n", host16);
      break;      
    case(ETHERNET_TYPE_IPV6):
      DEBUG("\t type = 0x%x : IPv6\n", host16);
      break;      
    default:
      DEBUG("\t hw_type = %x : UNKNOWN\n", host16);
  }
}

void print_arp_packet(struct arp_packet* pkt) {
  DEBUG("arp packet ------------------------------\n");
  uint16_t host16 = ntohs(pkt->hw_type);
  switch(host16) {
    case(ARP_HW_TYPE_ETHERNET):
      DEBUG("\t hw_type = %x : ETHERNET\n", host16);
      break;
    default:
      DEBUG("\t hw_type = %x : UNKNOWN\n", host16);
  }
  host16 = ntohs(pkt->pro_type);
  DEBUG("\t pro_type = 0x%x ; IP: 0x%x\n",  host16, ARP_PRO_TYPE_IPV4);
  DEBUG("\t hw_len = 0x%x; pro_len = 0x%x; IP: 0x%x\n",
        pkt->hw_len, pkt->pro_len);

  host16 = ntohs(pkt->opcode);
  switch(host16) {
    case(ARP_OPCODE_REQUEST):
      DEBUG("\t opcode = 0x%x : REQUEST\n", host16);
      break;
    case(ARP_OPCODE_REPLY):
      DEBUG("\t opcode = 0x%x : REPLY\n", host16);
      break;      
    default:
      DEBUG("\t opcode = 0x%x : UNKNOWN\n", host16);
  }
  char buf[25];
  memset(buf, 0, sizeof(buf));
  
  mac_inttostr(pkt->sender_hw_addr, buf, sizeof(buf));
  DEBUG("\t sender_hw_addr: %s\n",buf);

  ip_inttostr(ntohl(pkt->sender_ip_addr), buf, sizeof(buf));
  DEBUG("\t sender_ip_addr: %s 0x%x\n", buf, pkt->sender_ip_addr);

  mac_inttostr(pkt->target_hw_addr, buf, sizeof(buf));  
  DEBUG("\t target_hw_addr: %s\n", buf);

  ip_inttostr(ntohl(pkt->target_ip_addr), buf, sizeof(buf));
  DEBUG("\t target_ip_addr: %s (0x%x) \n", buf,  pkt->target_ip_addr);  
}

void arp_thread(void *in, void **out) {
  DEBUG("|arp_thread function ------------------------------|\n");
  struct arp_info * ai = (struct arp_info *)in;
  struct nk_net_dev_characteristics c;
  DEBUG("|get_characteristics function ------------------------------|\n");  
  DEBUG("|ai = %p\n",ai);
  nk_net_dev_get_characteristics(ai->netdev, &c);
  // now you have the MAC address in c
  // and the ip address in ai, so we have the binding once again

  uint8_t input_packet[c.max_tu];
  DEBUG("Can receive %d bytes\n", c.max_tu);
  DEBUG("arp_thread before while ------------------------------\n");
  while (1) { 
    // wait to receive a packet - should check errors
    DEBUG("arp_thread while loop ------------------------------\n");
    nk_net_dev_receive_packet(ai->netdev, input_packet,
                              c.max_tu, NK_DEV_REQ_BLOCKING);
    dump_packet(input_packet, 24);
    struct ethernet_header *eth_hdr = (struct ethernet_header*) input_packet;
    struct arp_packet *arp_pkt = (struct arp_packet*) (input_packet+sizeof(struct ethernet_header));
    // if(input packet is an ARP packet and its a request && it matches ai->ip_addr)
    print_ethernet_packet(eth_hdr); 
    if ((ntohs(eth_hdr->type) != ETHERNET_TYPE_ARP)) {
      DEBUG("Not an ARP packet (type = 0x%04x)\n", ntohs(eth_hdr->type));
      continue;
    }
    
    print_arp_packet(arp_pkt);
    // TODO: compare function: compare(arp_pkt, eth_hdr)
    // create eth pck function, crc function/overload
    // find the e1000
    if ((compare_mac((uint8_t *) arp_pkt->target_hw_addr,
                     (uint8_t *) ARP_BROADCAST_MAC) ||
         compare_mac((uint8_t *) arp_pkt->target_hw_addr, c.mac)) &&
        (arp_pkt->opcode == ARP_OPCODE_REQUEST) &&
        (arp_pkt->target_ip_addr == ai->ip_addr)) {
      uint8_t output_packet[c.max_tu];
      // construct an ARP reply in output_packet
      struct arp_packet *op_pkt = (struct arp_packet*) output_packet;
      create_arp_pkt(op_pkt, ARP_OPCODE_REPLY,
                     ai->ip_addr, c.mac, 
                     (uint32_t) arp_pkt->sender_ip_addr,
                     (uint8_t *) arp_pkt->sender_hw_addr);
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

  if (!ai->netdev) { 
      ERROR("Cannot find device\n");
      return -1;
  }

  DEBUG("ip_strtoint ========================================\n");
  ai->ip_addr = ip_strtoint(IP_ADDRESS_STRING);
  char buf[20];
  memset(buf,0, sizeof(buf));
  DEBUG("ip address string: %s, ip address uint32_t: 0x%08x, ip address string: \n",
        IP_ADDRESS_STRING, ai->ip_addr);//, ip_inttostr(ai->ip_addr, buf, 20));
  /* // keep the ip address in network byte order */
  DEBUG("ai=%p\n",ai);
  DEBUG("nk_thread_start ========================================\n");  
  if (nk_thread_start(arp_thread, (void*)ai , NULL, 1, PAGE_SIZE_4KB, &tid, 1)) {
    free(ai);
    return -1;
  } else {
    return 0;
  }
}

int arp_deinit()
{
  INFO("deinited\n");
  return 0;
}
