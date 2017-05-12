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

inline uint16_t hton16(uint16_t v) {
  return (v >> 8) | (v << 8);
}

inline uint32_t hton32(uint32_t v) {
  return hton16(v >> 16) | (hton16((uint16_t) v) << 16);
}

inline uint64_t hton64(uint64_t v) {
  return hton32(v >> 32) | ((uint64_t) hton32((uint32_t) v) << 32);
}

inline uint16_t ntoh16(uint16_t v) {
  return hton16(v);
}

inline uint32_t ntoh32(uint32_t v) {
  return hton32(v);
}

inline uint64_t ntoh64(uint64_t v) {
  return hton64(v);
}

// convert a string of an ip address to uint32_t
uint32_t ip_strtoint(char* str) {
  uint8_t a, b, c, d;
  sscanf(str, "%hhu.%hhu.%hhu.%hhu", &a, &b, &c, &d );
  return  ( a << 24 ) | ( b << 16 ) | ( c << 8 ) | d;
}

// convert uint32_t representation of an ip address to
// a dot-decimal representation
void ip_inttostr(uint32_t ipv4, char *buf, int len) {
  if(len < 16) return;
  uint8_t *ptr = (uint8_t*)&ipv4;
  sprintf(buf, "%hhu.%hhu.%hhu.%hhu", ptr[3], ptr[2], ptr[1], ptr[0]);
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

inline struct ip_header* get_ip_header(uint8_t* pkt) {
  return (struct ip_header*)(pkt + sizeof(struct eth_header));
}

inline struct icmp_header* get_icmp_header(uint8_t* pkt) {
  return (struct icmp_header*)(pkt + sizeof(struct eth_header) + sizeof(struct ip_header));
}

inline uint8_t* get_icmp_data(uint8_t* pkt) {
  return pkt + sizeof(struct eth_header) + sizeof(struct ip_header) + sizeof(struct icmp_header);
}

inline struct arp_header* get_arp_header(uint8_t* pkt) {
  return (struct arp_header*)(pkt + sizeof(struct eth_header));
}

inline uint8_t* get_arp_data(uint8_t* pkt) {
  return pkt + sizeof(struct eth_header) + sizeof(struct arp_header);
}

inline struct udp_header* get_udp_header(uint8_t* pkt) {
  return (struct udp_header*)(pkt + sizeof(struct eth_header) + sizeof(struct ip_header));
}

inline uint8_t* get_udp_data(uint8_t* pkt) {
  return pkt + sizeof(struct eth_header) + sizeof(struct ip_header) + sizeof(struct udp_header);
}

void create_eth_header(uint8_t *pkt, uint8_t *dst_mac,
                       uint8_t *src_mac, uint16_t ethtype) {
  struct eth_header* eth_hdr = (struct eth_header*) pkt;
  memcpy(eth_hdr->dst_mac, dst_mac, MAC_LEN);
  memcpy(eth_hdr->src_mac, src_mac, MAC_LEN);
  eth_hdr->eth_type = ethtype;
}

void create_arp_header(uint8_t *pkt, uint16_t opcode,
                        uint8_t *sender_mac, uint32_t sender_ip,
                        uint8_t *target_mac, uint32_t target_ip) {
  struct arp_header *arp_hdr = (struct arp_header *) pkt;
  arp_hdr->hw_type = hton16(ARP_HW_TYPE_ETHERNET);
  arp_hdr->pro_type = hton16(ARP_PRO_TYPE_IPV4);
  arp_hdr->hw_len = MAC_LEN;
  arp_hdr->pro_len = IPV4_LEN;
  arp_hdr->opcode = hton16(opcode);
  arp_hdr->sender_ip_addr = hton32(sender_ip);
  memcpy((void*)arp_hdr->sender_mac, sender_mac, MAC_LEN);
  arp_hdr->target_ip_addr = hton32(target_ip);
  switch (opcode) {
    case ARP_OPCODE_REPLY:
      memcpy((void*)arp_hdr->target_mac, target_mac, MAC_LEN);
      break;
    case ARP_OPCODE_REQUEST:
      // In an ARP request this field is ignored.
      memset((void*)arp_hdr->target_mac, 0, MAC_LEN);
      break;
  }
}

void create_arp_response(uint8_t *pkt, uint8_t *dst_mac,
                         uint32_t dst_ip_addr, uint8_t *src_mac,
                         uint32_t src_ip_addr) {
  uint8_t* eth_hdr = pkt;
  uint8_t* arp_pkt = pkt + sizeof(struct eth_header);
  create_eth_header(eth_hdr, dst_mac, src_mac, hton16(ETHERNET_TYPE_ARP));
  create_arp_header(arp_pkt, ARP_OPCODE_REPLY,
                    src_mac, src_ip_addr, dst_mac, dst_ip_addr);
}

void create_icmp_header(uint8_t* pkt, uint8_t type,
                        uint8_t code, uint16_t id, uint16_t seq_num) {
  struct icmp_header *icmp_hdr = (struct icmp_header *) pkt;
  icmp_hdr->type = type;
  icmp_hdr->code = code;
  icmp_hdr->id = ntoh16(id);
  icmp_hdr->seq_num = ntoh16(seq_num);
  icmp_hdr->checksum = checksum((void *)pkt, sizeof(struct icmp_header) + 48 + 8);
}

void create_ip_header(uint8_t *pkt, uint32_t dst_ip_addr,
                      uint32_t src_ip_addr, uint8_t protocol,
                      uint16_t data_len) {
  struct ip_header* ip_hdr = (struct ip_header*) pkt;
  ip_hdr->hl = IP_HEADER_LEN/4;
  ip_hdr->version = IP_VER_IPV4;
  ip_hdr->tos = 0x00;              /* unused */
  /* size of the datagram = size of ip header + data */
  ip_hdr->len = hton16(sizeof(struct ip_header) + data_len);
  ip_hdr->id = 0;
  ip_hdr->offset = hton16(IP_FLAG_DF);
  ip_hdr->ttl = IP_TTL;                /* from icmp of ping command */
  ip_hdr->protocol = protocol;
  ip_hdr->ip_src = hton32(src_ip_addr);
  ip_hdr->ip_dst = hton32(dst_ip_addr);
  ip_hdr->checksum = checksum((void*) ip_hdr, IP_HEADER_LEN);
}

void create_icmp_response(uint8_t *pkt, uint8_t *dst_mac,
                          uint32_t dst_ip_addr, uint8_t *src_mac,
                          uint32_t src_ip_addr, struct icmp_header *icmp_in) {
  uint8_t *eth_hdr = pkt;
  uint8_t *ip_hdr = pkt + sizeof(struct eth_header);
  uint8_t *icmp_hdr = ip_hdr + sizeof(struct ip_header);
  create_eth_header(eth_hdr, dst_mac, src_mac, hton16(ETHERNET_TYPE_IPV4));
  create_ip_header(ip_hdr, dst_ip_addr, src_ip_addr, IP_PRO_ICMP, 64);
  create_icmp_header(icmp_hdr, ICMP_ECHO_REPLY, 0,
                     hton16(icmp_in->id), hton16(icmp_in->seq_num));
}

void create_udp_header(uint8_t *pkt, uint16_t src_port, uint16_t dst_port,
                       uint16_t len) {
  struct udp_header* udp_pkt = (struct udp_header*) pkt;
  udp_pkt->src_port = hton16(src_port);
  udp_pkt->dst_port = hton16(dst_port);
  udp_pkt->length = hton16(len + sizeof(struct udp_header));
  udp_pkt->checksum = 0;
}

void create_udp_response(uint8_t *pkt, uint8_t *dst_mac,
                         uint32_t dst_ip_addr, uint8_t *src_mac,
                         uint32_t src_ip_addr, uint16_t data_len,
                         struct udp_header* udp_hdr_in) {
  uint8_t *eth_hdr = pkt;
  uint8_t *ip_hdr = pkt + sizeof(struct eth_header);
  uint8_t *udp_hdr = ip_hdr + sizeof(struct ip_header);
  create_eth_header(eth_hdr, dst_mac, src_mac, hton16(ETHERNET_TYPE_IPV4));
  create_udp_header(udp_hdr, hton16(udp_hdr_in->dst_port), 6002, data_len);
  create_ip_header(ip_hdr, dst_ip_addr, src_ip_addr, IP_PRO_UDP,
                   data_len + sizeof(struct udp_header));
}

void mac_inttostr(uint8_t *mac, char* buf, int len) {
  if(buf && len < 24)
    return;

  sprintf(buf, "%02x:%02x:%02x_%02x:%02x:%02x",
          mac[0],mac[1],mac[2], mac[3],mac[4],mac[5]);
}

char to_hex(uint8_t n) {
  n &= 0xf;
  if (n<10) {
    return '0'+n;
  } else {
    return 'a'+(n-10);
  }
}

void dump_packet(uint8_t *p, int len) {
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
      buf[3*i+2] = '_';
  }
  buf[len*3] = 0;

  DEBUG("Dump packet content %d bytes: \n", len);
  DEBUG("%s\n", buf);
}

void print_eth_header(struct eth_header* pkt) {
  DEBUG("eth packet ------------------------------\n");
  char buf[25];
  memset(buf, 0, sizeof(buf));

  mac_inttostr(pkt->dst_mac, buf, sizeof(buf));
  DEBUG("\t dst_addr: %s\n",buf);

  mac_inttostr(pkt->src_mac, buf, sizeof(buf));
  DEBUG("\t src_addr: %s\n",buf);

  uint16_t host16 = ntoh16(pkt->eth_type);
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

void print_arp_header(struct arp_header* pkt) {
  DEBUG("arp packet ------------------------------\n");
  uint16_t host16 = ntoh16(pkt->hw_type);
  switch(host16) {
    case(ARP_HW_TYPE_ETHERNET):
      DEBUG("\t hw_type = %04x : ETHERNET\n", host16);
      break;
    default:
      DEBUG("\t hw_type = %04x : UNKNOWN\n", host16);
  }
  host16 = ntoh16(pkt->pro_type);
  DEBUG("\t pro_type = 0x%04x ; protocol type IPv4: 0x%04x\n",
        host16, ARP_PRO_TYPE_IPV4);
  DEBUG("\t hw_len = 0x%x; pro_len = 0x%x\n",
        pkt->hw_len, pkt->pro_len);

  host16 = ntoh16(pkt->opcode);
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

  ip_inttostr(ntoh32(pkt->sender_ip_addr), buf, sizeof(buf));
  DEBUG("\t sender_ip_addr: %s (0x%x)\n", buf, pkt->sender_ip_addr);

  mac_inttostr(pkt->target_mac, buf, sizeof(buf));
  DEBUG("\t target_mac: %s\n", buf);

  ip_inttostr(ntoh32(pkt->target_ip_addr), buf, sizeof(buf));
  DEBUG("\t target_ip_addr: %s (0x%x) \n", buf,  pkt->target_ip_addr);
}

void print_arp_packet(uint8_t* pkt) {
  print_eth_header((struct eth_header*) pkt);
  print_arp_header(get_arp_header(pkt));
}

void print_ip_header(struct ip_header* pkt) {
  DEBUG("ip header ------------------------------\n");
  uint16_t host16 = 0;
  DEBUG("\t header length: 0x%02x\n", pkt->hl);
  DEBUG("\t version: 0x%02x\n", pkt->version);
  DEBUG("\t type of service: 0x%02x\n", pkt->tos);
  DEBUG("\t total length: %d\n", ntoh16(pkt->len));
  DEBUG("\t id: %d 0x%04x\n", ntoh16(pkt->id));
  DEBUG("\t flag: 0x%04x\n", pkt->offset & IP_FLAG_MASK);
  DEBUG("\t offset: 0x%04x\n", ntoh16(pkt->offset));
  DEBUG("\t time to live: 0x%02x\n", pkt->ttl);

  switch(pkt->protocol) {
    case IP_PRO_ICMP:
      DEBUG("\t protocol: 0x%02x icmp: 0x%02x\n", pkt->protocol, IP_PRO_ICMP);
      break;
    case IP_PRO_UDP:
      DEBUG("\t protocol: 0x%02x udp: 0x%02x\n", pkt->protocol, IP_PRO_UDP);
      break;
    default:
      DEBUG("\t protocol: 0x%02x unknown\n", pkt->protocol);
      break;
  }

  DEBUG("\t checksum: 0x%04x cal checksum 0x%04x\n",
        pkt->checksum, checksum((void *)pkt, sizeof(struct ip_header)));
  char buf[25];
  memset(buf, 0, sizeof(buf));

  ip_inttostr(ntoh32(pkt->ip_src), buf, sizeof(buf));
  DEBUG("\t ip src %s\n", buf);

  ip_inttostr(ntoh32(pkt->ip_dst), buf, sizeof(buf));
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
  DEBUG("\t id: 0x%04x\n", ntoh16(pkt->id));
  DEBUG("\t seq_num: 0x%04x\n", ntoh16(pkt->seq_num));
}

void print_icmp_packet(uint8_t* pkt) {
  print_eth_header((struct eth_header*)pkt);
  print_ip_header(get_ip_header(pkt));
  print_icmp_header(get_icmp_header(pkt));
}

void print_udp_header(struct udp_header* pkt) {
  DEBUG("udp message -----------------------------------\n");
  DEBUG("\t src port %d\n", ntoh16(pkt->src_port));
  DEBUG("\t dst port %d\n", ntoh16(pkt->dst_port));
  uint16_t pkt_len = ntoh16(pkt->length);
  DEBUG("\t length %d\n", pkt_len);
  DEBUG("\t checksum 0x%04x\n", ntoh16(pkt->checksum));
  char* pkt_data = (uint8_t *)pkt + sizeof(*pkt);
  pkt_data[pkt_len+1] = '\0';
  DEBUG("\t data: %s\n", pkt_data);
}

void print_udp_packet(uint8_t* pkt) {
  print_eth_header((struct eth_header*)pkt);
  print_ip_header(get_ip_header(pkt));
  print_udp_header(get_udp_header(pkt));
}

void arp_thread(void *in, void **out) {
  DEBUG("arp_thread function ------------------------------|\n");
  struct arp_info * ai = (struct arp_info *)in;
  DEBUG("ai = 0x%p\n", ai);
  struct nk_net_dev_characteristics c;
  DEBUG("calling nk_net_dev_get_characteristics &c: 0x%p|\n", &c);
  nk_net_dev_get_characteristics(ai->netdev, &c);
  uint32_t buffer_size = 2.5*1024;
  uint8_t input_packet[buffer_size];
  uint8_t output_packet[buffer_size];
  DEBUG("arp_thread before while ------------------------------\n");
  while (1) {
    memset(input_packet, 0, buffer_size);
    memset(output_packet, 0, buffer_size);  
    // wait to receive a packet - should check errors
    DEBUG("arp_thread while receiving ------------------------------\n");
    nk_net_dev_receive_packet(ai->netdev, input_packet,
                              c.max_tu, NK_DEV_REQ_BLOCKING);
    dump_packet(input_packet, 24);
    struct eth_header *eth_hdr_in = (struct eth_header*) input_packet;
    // if(input packet is an ARP packet its a request && it matches ai->ip_addr)
    DEBUG("ETHERNET_TYPE_ARP: %d\n",
          ntoh16(eth_hdr_in->eth_type) == ETHERNET_TYPE_ARP);
    DEBUG("if it is my packet ------------------------------\n");
    DEBUG("compare broadcast %d\n",
          compare_mac(eth_hdr_in->dst_mac, (uint8_t*) ARP_BROADCAST_MAC));
    DEBUG("compare dst %d\n",
          compare_mac(eth_hdr_in->dst_mac, c.mac));
    if (compare_mac(eth_hdr_in->dst_mac, (uint8_t*) ARP_BROADCAST_MAC) ||
        compare_mac(eth_hdr_in->dst_mac, c.mac)) {
      struct ip_header *ip_hdr_in = (struct ip_header *)(input_packet + sizeof(struct eth_header));
      if (ntoh16(eth_hdr_in->eth_type) == ETHERNET_TYPE_ARP) {
        /* DEBUG("ntoh16(arp_pkt_in->opcode) == ARP_OPCODE_REQUEST %d\n", */
        /*       ntoh16(arp_pkt_in->opcode) == ARP_OPCODE_REQUEST); */
        /* DEBUG("ntoh32(arp_pkt_in->target_ip_addr) == ai->ip_addr %d\n", */
        /*       ntoh32(arp_pkt_in->target_ip_addr) == ai->ip_addr);         */
        struct arp_header *arp_pkt_in = (struct arp_header*) (input_packet+sizeof(struct eth_header));
        if ((ntoh32(arp_pkt_in->target_ip_addr) == ai->ip_addr) &&
            (ntoh16(arp_pkt_in->opcode) == ARP_OPCODE_REQUEST)) {
          /* DEBUG("arp request packet ----------------------------------------\n"); */
          // construct an ARP reply in output_packet
          /* DEBUG("create an arp response\n"); */
          create_arp_response(output_packet, eth_hdr_in->src_mac,
                              ntoh32(arp_pkt_in->sender_ip_addr),
                              c.mac, ai->ip_addr);
          DEBUG("sending the arp response\n");
          nk_net_dev_send_packet(ai->netdev,
                                 output_packet,
                                 sizeof(struct eth_header) + sizeof(struct arp_header),
                                 NK_DEV_REQ_BLOCKING);
        }
      } else if (ntoh16(eth_hdr_in->eth_type) == ETHERNET_TYPE_IPV4) {
        /* DEBUG("ip protocol 0x%02x icmp protocol: 0x%02x \n", */
        /*       ip_hdr_in->protocol, IP_PRO_ICMP); */
        /* DEBUG("is my ip %d pkt_in->ip_src: 0x%04x my ip: 0x%04x\n", */
        /*       ntoh32(ip_hdr_in->ip_dst) == ai->ip_addr, */
        /*       ntoh32(ip_hdr_in->ip_dst), ai->ip_addr); */
        DEBUG("IP_PRO_ICMP: %d\n", ip_hdr_in->protocol == IP_PRO_ICMP);
        DEBUG("IP_PRO_UDP: %d\n", ip_hdr_in->protocol == IP_PRO_UDP);
        if (ip_hdr_in->protocol == IP_PRO_ICMP) {
          struct icmp_header *icmp_hdr_in = get_icmp_header(input_packet);
          DEBUG("is icmp request %d\n", icmp_hdr_in->type == ICMP_ECHO_REQUEST);
          if ((ntoh32(ip_hdr_in->ip_dst) == ai->ip_addr) &&
              (icmp_hdr_in->type == ICMP_ECHO_REQUEST)) {
            memcpy((void*) get_icmp_data(output_packet),
                   get_icmp_data(input_packet), 56);
            create_icmp_response(output_packet, eth_hdr_in->src_mac,
                                 ntoh32(ip_hdr_in->ip_src),
                                 c.mac, ai->ip_addr, icmp_hdr_in);
            print_icmp_packet(output_packet);
            nk_net_dev_send_packet(ai->netdev, output_packet,
                                   sizeof(struct eth_header) + sizeof(struct ip_header) + sizeof(struct icmp_header) + 56,
                                   NK_DEV_REQ_BLOCKING);
          }
        } else if (ip_hdr_in->protocol == IP_PRO_UDP) {
          DEBUG("receive UDP\n");
          DEBUG("print out the received udp packet\n");
          print_udp_packet(input_packet);
          char* udp_data_in = get_udp_data(input_packet);
          char* udp_data_out = get_udp_data(output_packet);
          uint32_t data_in_len = strlen(udp_data_in);
          memcpy(udp_data_out, udp_data_in, data_in_len);
          /* udp_data_out[0] = '|'; */
          /* udp_data_out[data_in_len-1] = '|'; */
          /* udp_data_out[data_in_len] = '\n';           */
          /* udp_data_out[data_in_len+1] =  '\0'; */
          create_udp_response(output_packet, eth_hdr_in->src_mac,
                              ntoh32(ip_hdr_in->ip_src),
                              c.mac, ai->ip_addr, strlen(udp_data_out),
                              get_udp_header(input_packet));
          DEBUG("finish creating the udop response\n");
          DEBUG("printing the response packet to debug\n");
          print_udp_packet(output_packet);
          nk_net_dev_send_packet(ai->netdev, output_packet,
                                 sizeof(struct eth_header) + sizeof(struct ip_header) + sizeof(struct udp_header) + strlen(udp_data_out),
                                 NK_DEV_REQ_BLOCKING);
        }
      }
    }
  }
}

int arp_init(struct naut_info * naut) {
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

int arp_deinit() {
  INFO("deinited\n");
  return 0;
}
