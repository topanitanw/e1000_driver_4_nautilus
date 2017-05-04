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
#include <nautilus/thread.h>                  // nk_start_thread
#include <nautilus/cpu.h>                     
#include <nautilus/naut_string.h>             // memset, memcpy
#include <nautilus/netdev.h>
#include <nautilus/printk.h>
#include <dev/pci.h>
#include <dev/e1000_pci.h>
#include <nautilus/arp.h>

#define NAUT_CONFIG_DEBUG_ARP 1
#ifndef NAUT_CONFIG_DEBUG_ARP
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...)
#endif

#define INFO(fmt, args...) INFO_PRINT("ARP INFO: " fmt, ##args)
#define DEBUG(fmt, args...) DEBUG_PRINT("ARP: " fmt, ##args)
#define ERROR(fmt, args...) ERROR_PRINT("ARP ERROR: " fmt, ##args)

const uint8_t ARP_BROADCAST_MAC[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

inline int compare_mac(uint8_t* mac1, uint8_t* mac2) {
  return ((mac1[0] == mac2[0]) && (mac1[1] == mac2[1]) && \
          (mac1[2] == mac2[2]) && (mac1[3] == mac2[3]) && \
          (mac1[4] == mac2[4]) && (mac1[5] == mac2[5]));
}

inline uint16_t htons(uint16_t v) {
  return (v >> 8) | (v << 8);
}

inline uint32_t htonl(uint32_t v) {
  return htons(v >> 16) | (htons((uint16_t) v) << 16);
}

inline uint64_t htonll(uint64_t v) {
  return htonl(v >> 32) | ((uint64_t) htonl((uint32_t) v) << 32);
}

inline uint16_t ntohs(uint16_t v) {
  return htons(v);
}

inline uint32_t ntohl(uint32_t v) {
  return htonl(v);
}

inline uint64_t ntohll(uint64_t v) {
  return htonll(v);
}

// convert a string of an ip address to uint32_t
uint32_t ip_strtoint(char* str) {
  uint8_t a, b, c, d;
  sscanf(str, "%hhu.%hhu.%hhu.%hhu", &a, &b, &c, &d );
  return  ( a << 24 ) | ( b << 16 ) | ( c << 8 ) | d;
}

uint16_t checksum(const void* addr, uint32_t len) {
  /* Compute Internet Checksum for "len" bytes
   *         beginning at location "addr".
   */
  uint32_t sum = 0;
  const uint16_t* buf = (uint16_t *) addr;
  while( len > 1 )  {
    /*  This is the inner loop */
    sum += * (uint16_t *) buf++;
    len -= 2;
  }

  /*  Add left-over byte, if any */
  if( len > 0 )
    sum += * (uint8_t *) buf;

  /*  Fold 32-bit sum to 16 bits */
  while (sum >> 16)
    sum = (sum & 0xffff) + (sum >> 16);
  
  return (uint16_t) ~sum;
}

// convert uint32_t representation of an ip address to
// a dot-decimal representation 
void ip_inttostr(uint32_t ipv4, char *buf, int len) {
  if(len < 16) return;
  uint8_t *ptr = (uint8_t*)&ipv4;
  sprintf(buf, "%hhu.%hhu.%hhu.%hhu", ptr[3], ptr[2], ptr[1], ptr[0]);
}

void create_eth_header(struct eth_header *hdr, uint8_t *dst_mac,
                       uint8_t *src_mac, uint16_t ethtype) {
  memcpy(hdr->dst_mac, dst_mac, MAC_LEN);
  memcpy(hdr->src_mac, src_mac, MAC_LEN);
  hdr->eth_type = ethtype;
}

void create_arp_content(struct arp_packet *pkt, uint16_t opcode,
                       uint8_t *sender_mac, uint32_t sender_ip, 
                       uint8_t *target_mac, uint32_t target_ip) {
  pkt->hw_type = htons(ARP_HW_TYPE_ETHERNET);
  pkt->pro_type = htons(ARP_PRO_TYPE_IPV4);
  pkt->hw_len = MAC_LEN;
  pkt->pro_len = IPV4_LEN;
  pkt->opcode = htons(opcode);
  pkt->sender_ip_addr = htonl(sender_ip);
  memcpy((void*)pkt->sender_mac, sender_mac, MAC_LEN);
  pkt->target_ip_addr = htonl(target_ip);
  switch (opcode) {
    case ARP_OPCODE_REPLY:
      memcpy((void*)pkt->target_mac, target_mac, MAC_LEN);
      break;
    case ARP_OPCODE_REQUEST:
      // In an ARP request this field is ignored.
      memset((void*)pkt->target_mac, 0, MAC_LEN);
      break;
  }
}

void create_arp_response(uint8_t *pkt, uint8_t *dst_mac,
                         uint32_t dst_ip_addr, uint8_t *src_mac,
                         uint32_t src_ip_addr) {
  struct eth_header *eth_hdr = (struct eth_header*) pkt;
  struct arp_packet *arp_pkt = (struct arp_packet*) (pkt + sizeof(struct eth_header));  
  create_eth_header(eth_hdr, dst_mac, src_mac, htons(ETHERNET_TYPE_ARP));
  create_arp_content(arp_pkt, ARP_OPCODE_REPLY,
                    src_mac, src_ip_addr, dst_mac, dst_ip_addr);
}

void create_icmp_header(struct icmp_header *pkt, uint8_t type,
                        uint8_t code, uint16_t id, uint16_t seq_num) {
  pkt->type = type;
  pkt->code = code;
  pkt->id = ntohs(id);
  pkt->seq_num = ntohs(seq_num);
  pkt->checksum = checksum((void *)pkt, sizeof(struct icmp_header));
}

void create_ip_header(struct ip_header *pkt, uint32_t dst_ip_addr,
                      uint32_t src_ip_addr, uint8_t protocol,
                      uint16_t data_len) {
  pkt->hl = IP_HEADER_LEN/4;
  pkt->version = IP_VER_IPV4;
  pkt->tos = 0x00;              /* unused */
  /* size of the datagram = size of ip header + data */
  pkt->len = ntohs(sizeof(struct ip_header) + data_len);
  pkt->id = 0;
  pkt->offset = 0;// ntohs(IP_FLAG_DF);
  pkt->ttl = 64;                /* from icmp of ping command */
  pkt->protocol = IP_PRO_ICMP;
  pkt->ip_src = ntohl(src_ip_addr);
  pkt->ip_dst = ntohl(dst_ip_addr);
  pkt->checksum = checksum((void*) pkt, IP_HEADER_LEN);
}

void create_icmp_response(uint8_t *pkt, uint8_t *dst_mac,
                          uint32_t dst_ip_addr, uint8_t *src_mac,
                          uint32_t src_ip_addr, struct icmp_header *icmp_in) {
  uint8_t *eth_hdr = pkt;  
  uint8_t *ip_hdr = pkt + sizeof(struct eth_header);
  uint8_t *icmp_hdr = pkt + sizeof(struct eth_header) + sizeof(struct ip_header);
  create_eth_header((struct eth_header *) eth_hdr, dst_mac, src_mac,
                    htons(ETHERNET_TYPE_IPV4));
  create_ip_header((struct ip_header *) ip_hdr, dst_ip_addr, src_ip_addr,
                   IP_PRO_ICMP,
                   84);
  create_icmp_header((struct icmp_header*) icmp_hdr, ICMP_ECHO_REPLY, 0,
                     ntohs(icmp_in->id), ntohs(icmp_in->seq_num));
}

void mac_inttostr(uint8_t *mac, char* buf, int len) {
  if(buf && len < 24)
    return;
  
  sprintf(buf, "%02x:%02x:%02x_%02x:%02x:%02x",
          mac[0],mac[1],mac[2], mac[3],mac[4],mac[5]);
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
    
  DEBUG("Dump packet content %d bytes: %s\n", len, buf);
}

void print_eth_header(struct eth_header* pkt) {
  DEBUG("eth packet ------------------------------\n");
  char buf[25];
  memset(buf, 0, sizeof(buf));
  
  mac_inttostr(pkt->dst_mac, buf, sizeof(buf));
  DEBUG("\t dst_addr: %s\n",buf);

  mac_inttostr(pkt->src_mac, buf, sizeof(buf));
  DEBUG("\t src_addr: %s\n",buf);
  
  uint16_t host16 = ntohs(pkt->eth_type);
  switch(host16) {
    case(ETHERNET_TYPE_ARP):
      DEBUG("\t type = 0x%04x : ARP\n", host16);
      break;
    case(ETHERNET_TYPE_IPV4):
      DEBUG("\t type = 0x%04x : IPv4\n", host16);
      break;
    case(ETHERNET_TYPE_IPX):
      DEBUG("\t type = 0x%x : IPx\n", host16);
      break;      
    case(ETHERNET_TYPE_IPV6):
      DEBUG("\t type = 0x%x : IPv6\n", host16);
      break;      
    default:
      DEBUG("\t type = 0x%x : UNKNOWN\n", host16);
  }
}

void print_arp_packet(struct arp_packet* pkt) {
  DEBUG("arp packet ------------------------------\n");
  uint16_t host16 = ntohs(pkt->hw_type);
  switch(host16) {
    case(ARP_HW_TYPE_ETHERNET):
      DEBUG("\t hw_type = %04x : ETHERNET\n", host16);
      break;
    default:
      DEBUG("\t hw_type = %04x : UNKNOWN\n", host16);
  }
  host16 = ntohs(pkt->pro_type);
  DEBUG("\t pro_type = 0x%04x ; protocol type IPv4: 0x%04x\n",
        host16, ARP_PRO_TYPE_IPV4);
  DEBUG("\t hw_len = 0x%x; pro_len = 0x%x\n",
        pkt->hw_len, pkt->pro_len);

  host16 = ntohs(pkt->opcode);
  switch(host16) {
    case(ARP_OPCODE_REQUEST):
      DEBUG("\t opcode = 0x%04x : REQUEST\n", host16);
      break;
    case(ARP_OPCODE_REPLY):
      DEBUG("\t opcode = 0x%04x : REPLY\n", host16);
      break;      
    default:
      DEBUG("\t opcode = 0x%04x : UNKNOWN\n", host16);
  }
  char buf[25];
  memset(buf, 0, sizeof(buf));
  
  mac_inttostr(pkt->sender_mac, buf, sizeof(buf));
  DEBUG("\t sender_mac: %s\n",buf);

  ip_inttostr(ntohl(pkt->sender_ip_addr), buf, sizeof(buf));
  DEBUG("\t sender_ip_addr: %s (0x%x)\n", buf, pkt->sender_ip_addr);

  mac_inttostr(pkt->target_mac, buf, sizeof(buf));  
  DEBUG("\t target_mac: %s\n", buf);

  ip_inttostr(ntohl(pkt->target_ip_addr), buf, sizeof(buf));
  DEBUG("\t target_ip_addr: %s (0x%x) \n", buf,  pkt->target_ip_addr);  
}

void print_ip_header(struct ip_header* pkt) {
  DEBUG("ip header ------------------------------\n");
  uint16_t host16 = 0;
  DEBUG("\t header length: 0x%02x\n", pkt->hl);
  DEBUG("\t version: 0x%02x\n", pkt->version);
  DEBUG("\t type of service: 0x%02x\n", pkt->tos);
  DEBUG("\t total length: %d\n", ntohs(pkt->len));
  DEBUG("\t id: %d 0x%04x\n", ntohs(pkt->id));
  DEBUG("\t flag: 0x%04x\n", pkt->offset & IP_FLAG_MASK);
  DEBUG("\t offset: 0x%04x\n", pkt->offset);
  DEBUG("\t time to live: 0x%02x\n", pkt->ttl);      
  DEBUG("\t protocol: 0x%02x icmp 0x%02x\n", pkt->protocol, IP_PRO_ICMP);
  DEBUG("\t checksum: 0x%04x cal checksum 0x%04x\n",
        pkt->checksum, checksum((void *)pkt, sizeof(struct ip_header)));
  char buf[25];
  memset(buf, 0, sizeof(buf));

  ip_inttostr(ntohl(pkt->ip_src), buf, sizeof(buf));
  DEBUG("\t ip src %s\n", buf);

  ip_inttostr(ntohl(pkt->ip_dst), buf, sizeof(buf));
  DEBUG("\t ip dst %s\n", buf);
}

void print_icmp_header(struct icmp_header* pkt) {
  DEBUG("icmp message ------------------------------\n");
  switch(pkt->type) {
    case ICMP_ECHO_REPLY:
      DEBUG("\t type: 0x%02x echo reply: 0x%02x\n",
            pkt->type, ICMP_ECHO_REPLY);
      break;

    case ICMP_ECHO_REQUEST:
      DEBUG("\t type: 0x%02x echo request: 0x%02x\n",
            pkt->type, ICMP_ECHO_REQUEST);
      break;
    default:
      DEBUG("\t type: 0x%02x\n");
  }
      
  DEBUG("\t code: 0x%02x\n", pkt->code);
  DEBUG("\t checksum: 0x%04x cal 0x%04x\n",
        pkt->checksum, checksum((void *)pkt, sizeof(struct icmp_header)));
  DEBUG("\t id: 0x%04x\n", ntohs(pkt->id));
  DEBUG("\t seq_num: 0x%04x\n", ntohs(pkt->seq_num));
}

void arp_thread(void *in, void **out) {
  DEBUG("arp_thread function ------------------------------|\n");
  struct arp_info * ai = (struct arp_info *)in;
  DEBUG("ai = 0x%p\n", ai);
  struct nk_net_dev_characteristics c;
  DEBUG("calling nk_net_dev_get_characteristics &c: 0x%p|\n", &c);
  nk_net_dev_get_characteristics(ai->netdev, &c);
  uint8_t input_packet[c.max_tu];
  DEBUG("arp_thread before while ------------------------------\n");
  while (1) { 
    // wait to receive a packet - should check errors
    DEBUG("arp_thread while receiving ------------------------------\n");
    nk_net_dev_receive_packet(ai->netdev, input_packet,
                              c.max_tu, NK_DEV_REQ_BLOCKING);
    dump_packet(input_packet, 24);
    struct eth_header *eth_hdr_in = (struct eth_header*) input_packet;
    // if(input packet is an ARP packet its a request && it matches ai->ip_addr)
    print_eth_header(eth_hdr_in);
    DEBUG("if it is my packet ------------------------------\n");
    DEBUG("compare broadcast %d\n",
          compare_mac(eth_hdr_in->dst_mac, (uint8_t*) ARP_BROADCAST_MAC));
    DEBUG("compare dst %d\n",
          compare_mac(eth_hdr_in->dst_mac, c.mac));
    
    if (compare_mac(eth_hdr_in->dst_mac, (uint8_t*) ARP_BROADCAST_MAC) ||
        compare_mac(eth_hdr_in->dst_mac, c.mac)) {
      if (ntohs(eth_hdr_in->eth_type) == ETHERNET_TYPE_ARP) {
        /* DEBUG("ntohs(arp_pkt_in->opcode) == ARP_OPCODE_REQUEST %d\n", */
        /*       ntohs(arp_pkt_in->opcode) == ARP_OPCODE_REQUEST); */
        /* DEBUG("ntohl(arp_pkt_in->target_ip_addr) == ai->ip_addr %d\n", */
        /*       ntohl(arp_pkt_in->target_ip_addr) == ai->ip_addr);         */
        struct arp_packet *arp_pkt_in = (struct arp_packet*) (input_packet+sizeof(struct eth_header));
        if ((ntohl(arp_pkt_in->target_ip_addr) == ai->ip_addr) &&
            (ntohs(arp_pkt_in->opcode) == ARP_OPCODE_REQUEST)) {
          DEBUG("arp request packet ----------------------------------------\n");
          uint8_t output_packet[c.max_tu];
          // construct an ARP reply in output_packet
          struct eth_header *eth_hdr_out = (struct eth_header*) output_packet;
          struct arp_packet *arp_pkt_out = (struct arp_packet*) (output_packet + sizeof(struct eth_header));
          DEBUG("create an arp response\n");
          create_arp_response(output_packet, eth_hdr_in->src_mac,
                              ntohl(arp_pkt_in->sender_ip_addr),
                              c.mac, ai->ip_addr);
          print_eth_header(eth_hdr_out);
          print_arp_packet(arp_pkt_out);
          DEBUG("sending the arp response\n");
          nk_net_dev_send_packet(ai->netdev,
                                 output_packet,
                                 sizeof(struct eth_header) + sizeof(struct arp_packet),
                                 NK_DEV_REQ_BLOCKING);
        }
      } else if (ntohs(eth_hdr_in->eth_type) == ETHERNET_TYPE_IPV4) {
        struct ip_header *ip_hdr_in = (struct ip_header *)(input_packet + sizeof(struct eth_header));
        DEBUG("ip protocol 0x%02x icmp protocol: 0x%02x \n",
              ip_hdr_in->protocol, IP_PRO_ICMP);
        DEBUG("is my ip %d pkt_in->ip_src: 0x%04x my ip: 0x%04x\n",
              ntohl(ip_hdr_in->ip_dst) == ai->ip_addr,
              ntohl(ip_hdr_in->ip_dst), ai->ip_addr);
        struct icmp_header *icmp_hdr_in = (struct icmp_header *)(((uint8_t *)ip_hdr_in) + sizeof(struct ip_header));
        DEBUG("is icmp request %d\n",
              icmp_hdr_in->type == ICMP_ECHO_REQUEST);
        
        if ((ntohl(ip_hdr_in->ip_dst) == ai->ip_addr) &&
            (ip_hdr_in->protocol == IP_PRO_ICMP) &&
            (icmp_hdr_in->type == ICMP_ECHO_REQUEST)) {
          uint8_t output_packet[c.max_tu];
          struct eth_header *eth_hdr_out = (struct eth_header *) output_packet;
          struct ip_header *ip_pkt_out = (struct ip_header *) (output_packet + sizeof(struct eth_header));
          struct icmp_header *icmp_hdr_out = (struct icmp_header *) (output_packet + sizeof(struct eth_header) + sizeof(struct ip_header));
          DEBUG("printing packet before tx\n");
          char buf[30];
          memset(buf, 0, sizeof(buf));
          mac_inttostr(eth_hdr_in->src_mac, buf, sizeof(buf));
          DEBUG("dst_mac: %s\n", buf);
          create_icmp_response(output_packet, eth_hdr_in->src_mac,
                               ntohl(ip_hdr_in->ip_src),
                               c.mac, ai->ip_addr, icmp_hdr_in);
          memcpy((void*) ((uint8_t*)icmp_hdr_in + sizeof(struct icmp_header)),
                 (input_packet + sizeof(struct eth_header) + sizeof(struct ip_header) + sizeof(struct icmp_header)), 56);
          print_eth_header(eth_hdr_out);
          print_ip_header(ip_pkt_out);
          print_icmp_header(icmp_hdr_out);          
          nk_net_dev_send_packet(ai->netdev,
                                 output_packet,
                                 sizeof(struct eth_header) + sizeof(struct ip_header)
                                 + sizeof(struct icmp_header),
                                 NK_DEV_REQ_BLOCKING);          
        }
      }
    }
  }
}

int arp_init(struct naut_info * naut)
{
  DEBUG("arp init ========================================\n");
  struct arp_info *ai = malloc(sizeof(*ai));
  if (!ai) {
    ERROR("Cannot allocate ai arp_info\n");
    return -1;
  }  
  // search for the device in netdev
  ai->netdev = nk_net_dev_find("e1000-0");

  if (!ai->netdev) { 
    ERROR("Cannot find the \"e1000-0\" ethernet adapter from nk_net_dev\n");
    return -1;
  }

  ai->ip_addr = ip_strtoint(IP_ADDRESS_STRING);
  char buf[20];
  memset(buf,0, sizeof(buf));
  // test the ip_inttostr function
  ip_inttostr(ai->ip_addr, buf, 20);
  DEBUG("ip address string: %s, ip address uint32_t: 0x%08x, string: %s\n",
        IP_ADDRESS_STRING, ai->ip_addr, buf);
  DEBUG("nk_thread_start ========================================\n");  
  nk_thread_id_t tid;
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
