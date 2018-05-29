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
 * http://xstack.sandia.gov/hobbes
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
#include <nautilus/thread.h>          // nk_start_thread
#include <nautilus/cpu.h>
#include <nautilus/naut_string.h>     // memset, memcpy
#include <nautilus/netdev.h>
#include <nautilus/printk.h>
#include <nautilus/mm.h>              // malloc
#include <dev/pci.h>
#include <nautilus/vc.h>              // nk_vc_printf
#include <nautilus/cpu.h>             // udelay
#include <dev/e1000e_pci.h>           // e1000e_interpret_int
#include <nautilus/cpu.h>             // rdtsc 

// #define DEBUG_ECHO 1

#ifndef DEBUG_ECHO
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...)
#endif

#define INFO(fmt, args...) INFO_PRINT("net_udp_echo: " fmt, ##args)
#define DEBUG(fmt, args...) DEBUG_PRINT("net_udp_echo: " fmt, ##args)
#define ERROR(fmt, args...) ERROR_PRINT(fmt, ##args)

uint8_t ARP_BROADCAST_MAC[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

// general definition
#define IPV4_LEN                  4              /* length of ipv4 in bytes */
#define MAC_INT_LEN                   6              /* length of mac address in bytes */
#define IP_ADDRESS_STRING         "10.10.10.3"
#define MAC_STRING_LEN            25             /* size of a string format of a mac address */
/* a string format of a mac address xx:xx:xx_xx:xx:xx\0 */
#define IP_STRING_LEN             17             /* size of a string format of an ip address */
#define MAC_R4154_E1000E          "68:05:CA:2D:A3:54"
#define MAC_R4155_E1000E          "68:05:CA:2D:A4:10"
#define MAC_R4156_E1000E          "68:05:CA:2D:A4:10"

#define MACHINE_NO_R4154          4
#define MACHINE_NO_R4155          5
#define MACHINE_NO_R4156          6

/* a string format of an ip address xxx.xxx.xxx.xxx\0 */

// ethernet frame
#define ETHERNET_TYPE_ARP         0x0806         /* ethernet type arp */
#define ETHERNET_TYPE_IPV4        0x0800         /* ethernet type ipv4 */
#define ETHERNET_TYPE_IPX         0x8137         /* ethernet type ipx */
#define ETHERNET_TYPE_IPV6        0x86dd         /* ethernet type ipv6 */
#define ETHERNET_TYPE_RUNT        0x5555         /* ethernet runt packet */
#define ETHERNET_TYPE_IRUNT       0x5556         /* ethernet runt packet */

// ip packet
// ip protocol
#define IP_HEADER_LEN             20
#define IP_VER_IPV4               4
#define IP_VER_IPV6               6
#define IP_TTL                    64             /* time to live */
#define IP_FLAG_MASK              0xE000
#define IP_FLAG_MF                0x2000         /* more fragment */
#define IP_FLAG_DF                0x4000         /* no fragment */
#define IP_PRO_ICMP               0x01           /* ICMP protocol */
#define IP_PRO_UDP                0x11           /* udp protocol */

// ARP frame
#define ARP_HW_TYPE_ETHERNET      1              /* arp hardware type for ethernet */
#define ARP_PRO_TYPE_IPV4         0x0800         /* arp protocol type for ipv4 */
#define ARP_OPCODE_REQUEST        0x0001         /* arp opcode request */
#define ARP_OPCODE_REPLY          0x0002         /* arp opcode reply */

// Internet Control Message Protocol (ICMP) message
#define ICMP_ECHO_REPLY           0
#define ICMP_ECHO_REQUEST         8

#define TX_REQ NK_DEV_REQ_BLOCKING

#define RX_REQ NK_DEV_REQ_BLOCKING

// ethernet header
// size = 18 bytes
struct eth_header {
  uint8_t  dst_mac[MAC_INT_LEN];        /* destination mac address */
  uint8_t  src_mac[MAC_INT_LEN];        /* source mac address */
  uint16_t eth_type;                /* ethernet type */
} __packed;
typedef struct eth_header eth_hdr_t;

struct runt_header {
  eth_hdr_t eth_hdr;
  uint32_t pkt_id;
} __packed;
typedef struct runt_header runt_hdr_t;

// IP header format
//  0                   1                   2                   3
//  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |Version|  IHL  |Type of Service|          Total Length         |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |         Identification        |Flags|      Fragment Offset    |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |  Time to Live |    Protocol   |         Header Checksum       |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |                       Source Address                          |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |                    Destination Address                        |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |                    Options                    |    Padding    |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// size 20 bytes
struct ip_header {
  uint8_t  hl : 4;                     /* header length */
  uint8_t  version : 4;                /* version */
  uint8_t  tos;                        /* type of service */
  uint16_t len;                        /* total length */
  uint16_t id;                         /* identification */
  uint16_t offset; 
  uint8_t ttl;                         /* time to live */
  uint8_t protocol;                    /* protocol */
  uint16_t checksum;                   /* checksum */
  uint32_t ip_src;                     /* source and dest address */
  uint32_t ip_dst;  
} __packed;
typedef struct ip_header ip_hdr_t;

// udp header
//  0              15  16             31
//  +--------+--------+--------+--------+
//  |     Source      |   Destination   |
//  |      Port       |      Port       |
//  +--------+--------+--------+--------+
//  |    Checksum     |                 |
//  |    Coverage     |    Checksum     |
//  +--------+--------+--------+--------+
//  :              Payload              :
//  +-----------------------------------+
// size 8 bytes
struct udp_header {
  uint16_t src_port;                   /* source port */
  uint16_t dst_port;                   /* destination port */
  uint16_t length;                     /* length */
  uint16_t checksum;                   /* checksum */
} __packed;

// Internet Protocol (IPv4) over Ethernet ARP packet
struct arp_header {
  uint16_t hw_type;                    /* hardware address */
  uint16_t pro_type;                   /* protocol address */
  uint8_t hw_len;                      /* hardware address length */
  uint8_t pro_len;                     /* protocol address length */
  uint16_t opcode;                     /* arp operation */
  uint8_t sender_mac[MAC_INT_LEN];         /* sender hardware address */
  uint32_t sender_ip_addr;             /* sender protocol address */
  uint8_t target_mac[MAC_INT_LEN];         /* target hardware address */
  uint32_t target_ip_addr;             /* target protocol address */
} __attribute__((packed));

// icmp header
// size = 8 without data
struct icmp_header {
  uint8_t type;                        /* type */
  uint8_t code;                        /* code */
  uint16_t checksum;                   /* icmp header checksum */
  uint16_t id;                         /* identifier */
  uint16_t seq_num;                    /* sequence number */
} __packed;

struct arp_packet {
  struct eth_header eth;
  struct arp_header arp;
} __packed;

struct udp_packet {
  struct eth_header eth;
  struct ip_header ip;
  struct udp_header udp;
  uint8_t data;
} __packed;

struct action_info {
  struct nk_net_dev *netdev;
  uint32_t nk_ip_addr;
  uint32_t dst_ip_addr;
  uint8_t dst_mac[MAC_INT_LEN];
  uint16_t port;
  char* nic_name;
  uint32_t packet_num;
  uint8_t vc;
};

struct data_item {
  uint64_t* arr;
  uint64_t total;
};
typedef struct data_item data_item_t;

struct data_collection {
  data_item_t tsc;
  data_item_t epkt; // error packet
  data_item_t rx_count;
  uint64_t size;
};
typedef struct data_collection data_collection_t;

typedef struct data_op {
  data_collection_t tx;
  data_collection_t rx;
} data_op_t;

struct buffer {
  uint8_t* buf;
  uint32_t size;
};
typedef struct buffer buf_t;

struct echo_info {
  struct action_info* ai; 
  struct nk_net_dev_characteristics dev_char;
  buf_t in;
  buf_t out;
};
typedef struct echo_info echo_info_t;

extern volatile uint64_t ps_sta;
extern volatile uint64_t ps_end;

extern volatile uint64_t pr_sta;
extern volatile uint64_t pr_end;

extern volatile iteration_t measure;

static inline int compare_mac(uint8_t* mac1, uint8_t* mac2) {
  return ((mac1[0] == mac2[0]) && (mac1[1] == mac2[1]) && \
          (mac1[2] == mac2[2]) && (mac1[3] == mac2[3]) && \
          (mac1[4] == mac2[4]) && (mac1[5] == mac2[5]));
}

static int mac_strtoint(uint8_t* mac_int, char* mac_str) {
  sint32_t temp[MAC_INT_LEN];
  if(MAC_INT_LEN == sscanf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x*c", 
                       &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5])) {
    
    for(uint8_t i = 0; i < MAC_INT_LEN; i++) {
      mac_int[i] = temp[i];
    }

    return 0;
  }
  return -1;
}

static void mac_inttostr(uint8_t *mac, char* buf, int len) {
  if(!buf || len < MAC_STRING_LEN)
    return;

  sprintf(buf, "%02x:%02x:%02x_%02x:%02x:%02x",
          mac[0],mac[1],mac[2], mac[3],mac[4],mac[5]);
}

static char to_hex(uint8_t n) {
  n &= 0xf;
  if (n<10) {
    return '0'+n;
  } else {
    return 'a'+(n-10);
  }
}

static void dump_packet(uint8_t *p, int len) {
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

  INFO("Dump packet content %d bytes: %s\n", len, buf);
}

static inline uint16_t hton16(uint16_t v) {
  return (v >> 8) | (v << 8);
}

static inline uint32_t hton32(uint32_t v) {
  return hton16(v >> 16) | (hton16((uint16_t) v) << 16);
}

static inline uint64_t hton64(uint64_t v) {
  return hton32(v >> 32) | ((uint64_t) hton32((uint32_t) v) << 32);
}

static inline uint16_t ntoh16(uint16_t v) {
  return hton16(v);
}

static inline uint32_t ntoh32(uint32_t v) {
  return hton32(v);
}

static inline uint64_t ntoh64(uint64_t v) {
  return hton64(v);
}

// convert a string of an ip address to uint32_t
static uint32_t ip_strtoint(char* str) {
  uint8_t a, b, c, d;
  sscanf(str, "%hhu.%hhu.%hhu.%hhu", &a, &b, &c, &d );
  return  ( a << 24 ) | ( b << 16 ) | ( c << 8 ) | d;
}

// convert uint32_t representation of an ip address to
// a dot-decimal representation
static void ip_inttostr(uint32_t ipv4, char *buf, int len) {
  if(len < IP_STRING_LEN) return;
  uint8_t *ptr = (uint8_t*)&ipv4;
  sprintf(buf, "%hhu.%hhu.%hhu.%hhu", ptr[3], ptr[2], ptr[1], ptr[0]);
}

static uint16_t checksum(const void* addr, uint32_t len) {
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

static inline struct ip_header* get_ip_header(uint8_t* pkt) {
  return (struct ip_header*)(pkt + sizeof(struct eth_header));
}

static inline struct icmp_header* get_icmp_header(uint8_t* pkt) {
  return (struct icmp_header*)(pkt + sizeof(struct eth_header) + sizeof(struct ip_header));
}

static inline uint8_t* get_icmp_data(uint8_t* pkt) {
  return pkt + sizeof(struct eth_header) + sizeof(struct ip_header) + sizeof(struct icmp_header);
}

static inline struct arp_header* get_arp_header(uint8_t* pkt) {
  return (struct arp_header*)(pkt + sizeof(struct eth_header));
}

static inline uint8_t* get_arp_data(uint8_t* pkt) {
  return pkt + sizeof(struct eth_header) + sizeof(struct arp_header);
}

static inline struct udp_header* get_udp_header(uint8_t* pkt) {
  return (struct udp_header*)(pkt + sizeof(struct eth_header) + sizeof(struct ip_header));
}

static inline uint8_t* get_udp_data(uint8_t* pkt) {
  return pkt + sizeof(struct eth_header) + sizeof(struct ip_header) + sizeof(struct udp_header);
}

static void create_eth_header(uint8_t *pkt, uint8_t *dst_mac,
                              uint8_t *src_mac, uint16_t ethtype) {
  struct eth_header* eth_hdr = (struct eth_header*) pkt;
  memcpy(eth_hdr->dst_mac, dst_mac, MAC_INT_LEN);
  memcpy(eth_hdr->src_mac, src_mac, MAC_INT_LEN);
  eth_hdr->eth_type = hton16(ethtype);
}

static void create_arp_header(uint8_t *pkt, uint16_t opcode,
                        uint8_t *sender_mac, uint32_t sender_ip,
                        uint8_t *target_mac, uint32_t target_ip) {
  struct arp_header *arp_hdr = (struct arp_header *) pkt;
  arp_hdr->hw_type = hton16(ARP_HW_TYPE_ETHERNET);
  arp_hdr->pro_type = hton16(ARP_PRO_TYPE_IPV4);
  arp_hdr->hw_len = MAC_INT_LEN;
  arp_hdr->pro_len = IPV4_LEN;
  arp_hdr->opcode = hton16(opcode);
  arp_hdr->sender_ip_addr = hton32(sender_ip);
  memcpy((void*)arp_hdr->sender_mac, sender_mac, MAC_INT_LEN);
  arp_hdr->target_ip_addr = hton32(target_ip);
  switch (opcode) {
    case ARP_OPCODE_REPLY:
      memcpy((void*)arp_hdr->target_mac, target_mac, MAC_INT_LEN);
      break;
    case ARP_OPCODE_REQUEST:
      // In an ARP request this field is ignored.
      memset((void*)arp_hdr->target_mac, 0, MAC_INT_LEN);
      break;
  }
}

static void create_arp_response(uint8_t *pkt, uint8_t *dst_mac,
                         uint32_t dst_ip_addr, uint8_t *src_mac,
                         uint32_t src_ip_addr) {
  uint8_t* eth_hdr = pkt;
  uint8_t* arp_pkt = pkt + sizeof(struct eth_header);
  create_eth_header(eth_hdr, dst_mac, src_mac, ETHERNET_TYPE_ARP);
  create_arp_header(arp_pkt, ARP_OPCODE_REPLY,
                    src_mac, src_ip_addr, dst_mac, dst_ip_addr);
}

static void create_icmp_header(uint8_t* pkt, uint8_t type,
                        uint8_t code, uint16_t id, uint16_t seq_num) {
  struct icmp_header *icmp_hdr = (struct icmp_header *) pkt;
  icmp_hdr->type = type;
  icmp_hdr->code = code;
  icmp_hdr->id = ntoh16(id);
  icmp_hdr->seq_num = ntoh16(seq_num);
  icmp_hdr->checksum = checksum((void *)pkt, sizeof(struct icmp_header) + 48 + 8);
}

static void create_ip_header(uint8_t *pkt, uint32_t dst_ip_addr,
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

static void create_icmp_response(uint8_t *pkt, uint8_t *dst_mac,
                          uint32_t dst_ip_addr, uint8_t *src_mac,
                          uint32_t src_ip_addr, struct icmp_header *icmp_in) {
  uint8_t *eth_hdr = pkt;
  uint8_t *ip_hdr = pkt + sizeof(struct eth_header);
  uint8_t *icmp_hdr = ip_hdr + sizeof(struct ip_header);
  create_eth_header(eth_hdr, dst_mac, src_mac, ETHERNET_TYPE_IPV4);
  create_ip_header(ip_hdr, dst_ip_addr, src_ip_addr, IP_PRO_ICMP, 64);
  create_icmp_header(icmp_hdr, ICMP_ECHO_REPLY, 0,
                     hton16(icmp_in->id), hton16(icmp_in->seq_num));
}

static void create_udp_header(uint8_t *pkt, uint16_t src_port, uint16_t dst_port,
                              uint16_t len) {
  struct udp_header* udp_pkt = (struct udp_header*) pkt;
  udp_pkt->src_port = hton16(src_port);
  udp_pkt->dst_port = hton16(dst_port);
  udp_pkt->length = hton16(len + sizeof(struct udp_header));
  udp_pkt->checksum = 0;
}

static void create_udp_response(uint8_t *pkt, uint8_t *dst_mac,
                                uint32_t dst_ip_addr, uint8_t *src_mac,
                                uint32_t src_ip_addr, uint16_t data_len,
                                struct udp_header* udp_hdr_in,
                                struct action_info *ai) {
  uint8_t *eth_hdr = pkt;
  uint8_t *ip_hdr = pkt + sizeof(struct eth_header);
  uint8_t *udp_hdr = ip_hdr + sizeof(struct ip_header);
  create_eth_header(eth_hdr, dst_mac, src_mac, ETHERNET_TYPE_IPV4);
  create_udp_header(udp_hdr, hton16(udp_hdr_in->dst_port),
                    hton16(udp_hdr_in->src_port), data_len);
  create_ip_header(ip_hdr, dst_ip_addr, src_ip_addr, IP_PRO_UDP,
                   data_len + sizeof(struct udp_header));
}

static void print_eth_header(struct eth_header* pkt) {
  DEBUG("eth packet ------------------------------\n");
  char buf[25];
  memset(buf, 0, sizeof(buf));

  mac_inttostr(pkt->dst_mac, buf, sizeof(buf));
  INFO("\t dst_addr: %s\n",buf);

  mac_inttostr(pkt->src_mac, buf, sizeof(buf));
  INFO("\t src_addr: %s\n",buf);

  uint16_t host16 = ntoh16(pkt->eth_type);
  switch(host16) {
    case(ETHERNET_TYPE_ARP):
      INFO("\t type = 0x%04x : ARP\n", host16);
      break;
    case(ETHERNET_TYPE_IPV4):
      INFO("\t type = 0x%04x : IPv4\n", host16);
      break;
    case(ETHERNET_TYPE_IPX):
      INFO("\t type = 0x%x : IPx\n", host16);
      break;
    case(ETHERNET_TYPE_IPV6):
      INFO("\t type = 0x%x : IPv6\n", host16);
      break;
    default:
      INFO("\t type = 0x%x : UNKNOWN\n", host16);
  }
}


static void print_arp_header(struct arp_header* pkt) {
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

static void print_arp_packet(uint8_t* pkt) {
  print_eth_header((struct eth_header*) pkt);
  print_arp_header(get_arp_header(pkt));
}

static void print_arp_short(uint8_t* pkt) {
  struct arp_packet* arp_pkt = (struct arp_packet*) pkt;
  char buf[100];
  memset(buf, 0, sizeof(buf));
  char ip_buf[IP_STRING_LEN];
  ip_inttostr(ntoh32(arp_pkt->arp.sender_ip_addr), ip_buf, IP_STRING_LEN);
  sint64_t buf_offset = sprintf(buf, "ARP from ip: %s ", ip_buf);
  uint16_t host16 = ntoh16(arp_pkt->arp.opcode);
  switch(host16) {
    case(ARP_OPCODE_REQUEST):
      sprintf(&buf[buf_offset], "REQUEST");
      break;
    case(ARP_OPCODE_REPLY):
      sprintf(&buf[buf_offset], "REPLY");
      break;
    default:
      sprintf(&buf[buf_offset], "UNKNOWN");
  }  
  DEBUG(" %s\n", buf);  
}

static void print_ip_header(struct ip_header* pkt) {
  INFO("ip header ------------------------------\n");
  uint16_t host16 = 0;
  INFO("\t header length: 0x%02x\n", pkt->hl);
  INFO("\t version: 0x%02x\n", pkt->version);
  INFO("\t type of service: 0x%02x\n", pkt->tos);
  INFO("\t total length: %d\n", ntoh16(pkt->len));
  INFO("\t id: %d 0x%04x\n", ntoh16(pkt->id));
  INFO("\t flag: 0x%04x\n", pkt->offset & IP_FLAG_MASK);
  INFO("\t offset: 0x%04x\n", ntoh16(pkt->offset));
  INFO("\t time to live: 0x%02x\n", pkt->ttl);

  switch(pkt->protocol) {
    case IP_PRO_ICMP:
      INFO("\t protocol: 0x%02x icmp: 0x%02x\n", pkt->protocol, IP_PRO_ICMP);
      break;
    case IP_PRO_UDP:
      INFO("\t protocol: 0x%02x udp: 0x%02x\n", pkt->protocol, IP_PRO_UDP);
      break;
    default:
      INFO("\t protocol: 0x%02x unknown\n", pkt->protocol);
      break;
  }

  INFO("\t checksum: 0x%04x cal checksum 0x%04x\n",
        pkt->checksum, checksum((void *)pkt, sizeof(struct ip_header)));
  char buf[25];
  memset(buf, 0, sizeof(buf));

  ip_inttostr(ntoh32(pkt->ip_src), buf, sizeof(buf));
  INFO("\t ip src %s\n", buf);

  ip_inttostr(ntoh32(pkt->ip_dst), buf, sizeof(buf));
  INFO("\t ip dst %s\n", buf);
  return;
}

static void print_icmp_header(struct icmp_header* pkt) {
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

static void print_icmp_packet(uint8_t* pkt) {
  print_eth_header((struct eth_header*)pkt);
  print_ip_header(get_ip_header(pkt));
  print_icmp_header(get_icmp_header(pkt));
}

static void print_udp_header(struct udp_header* pkt) {
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

static void print_udp_packet(uint8_t* pkt) {
  print_eth_header((struct eth_header*)pkt);
  print_ip_header(get_ip_header(pkt));
  print_udp_header(get_udp_header(pkt));
}

static void print_udp_short(uint8_t* pkt) {
  struct udp_packet* udp_pkt = (struct udp_packet*) pkt;
  char buf[80];
  memset(buf, 0, sizeof(buf));
  char ip_buf[IP_STRING_LEN];
  ip_inttostr(ntoh32(udp_pkt->ip.ip_dst), ip_buf, IP_STRING_LEN);
  sprintf(buf, "UDP udp data len: %d ip_dst: %s src_port: %d\0",
          udp_pkt->udp.length, ip_buf, udp_pkt->udp.src_port);
  DEBUG(" %s\n", buf);
}

static int send_arp_response_packet(uint8_t* input_packet,
                                    uint8_t* output_packet,
                                    uint8_t* nic_mac,
                                    struct action_info* ai) {
  struct eth_header *eth_hdr_in = (struct eth_header*) input_packet;
  struct arp_header *arp_pkt_in = get_arp_header(input_packet);
  create_arp_response(output_packet, eth_hdr_in->src_mac,
                      ntoh32(arp_pkt_in->sender_ip_addr),
                      nic_mac, ai->nk_ip_addr);
  DEBUG("sending the arp response\n");
  return nk_net_dev_send_packet(ai->netdev,
                                output_packet,
                                sizeof(struct eth_header) + sizeof(struct arp_header),
                                TX_REQ,0,0);
}

static int send_icmp_packet(uint8_t* input_packet, uint8_t* output_packet,
                     uint8_t* nic_mac, struct action_info* ai) {
  struct eth_header *eth_hdr_in = (struct eth_header*) input_packet;
  struct icmp_header* icmp_hdr_in = get_icmp_header(input_packet);
  struct ip_header *ip_hdr_in = get_ip_header(input_packet);
  DEBUG("is icmp request %d\n", icmp_hdr_in->type == ICMP_ECHO_REQUEST);
  memcpy((void*) get_icmp_data(output_packet), get_icmp_data(input_packet), 56);
  create_icmp_response(output_packet, eth_hdr_in->src_mac,
                       ntoh32(ip_hdr_in->ip_src),
                       nic_mac, ai->nk_ip_addr, icmp_hdr_in);
  print_icmp_packet(output_packet);
  return nk_net_dev_send_packet(ai->netdev, output_packet,
                                sizeof(struct eth_header) + sizeof(struct ip_header) + sizeof(struct icmp_header) + 56,
                                TX_REQ,0,0);
}

static int send_udp_packet(uint8_t* input_packet, uint8_t* output_packet,
                           uint8_t* nic_mac, struct action_info* ai) {
  struct eth_header *eth_hdr_in = (struct eth_header*) input_packet;
  struct ip_header *ip_hdr_in = get_ip_header(input_packet);
  char* udp_data_in = get_udp_data(input_packet);
  char* udp_data_out = get_udp_data(output_packet);
  uint32_t data_in_len = strlen(udp_data_in);
  memcpy(udp_data_out, udp_data_in, data_in_len);
  create_udp_response(output_packet, eth_hdr_in->src_mac,
                      ntoh32(ip_hdr_in->ip_src),
                      nic_mac, ai->nk_ip_addr, data_in_len,
                      get_udp_header(input_packet), ai);
  DEBUG("finish creating the udop response\n");
  DEBUG("printing the response packet to debug\n");
  print_udp_packet(output_packet);
  return nk_net_dev_send_packet(ai->netdev, output_packet,
                                sizeof(struct eth_header) + sizeof(struct ip_header) + sizeof(struct udp_header) + strlen(udp_data_out),
                                TX_REQ,0,0);
}

void send_arp_request_packet(uint8_t* pkt,
                             uint8_t* nic_mac,
                             struct action_info* ai) {
  uint8_t* arp_pkt = pkt + sizeof(struct eth_header);
  create_eth_header(pkt, ARP_BROADCAST_MAC, nic_mac, ETHERNET_TYPE_ARP);
  create_arp_header(arp_pkt, ARP_OPCODE_REQUEST, nic_mac, ai->nk_ip_addr,
                    0, ai->dst_ip_addr);
  // print_arp_packet(pkt);
  DEBUG("sending arp request\n");
  nk_net_dev_send_packet(ai->netdev,
                         pkt,
                         sizeof(struct arp_packet),
                         TX_REQ,0,0);
  DEBUG("finish sending arp request\n");  
}

void print_runt_header(runt_hdr_t* pkt) {
  print_eth_header((eth_hdr_t*) pkt);
  INFO("\t pkt_id: %lu\n", pkt->pkt_id);
}

void print_packet(uint8_t* pkt) {
  eth_hdr_t* eth_hdr = (eth_hdr_t*) pkt;
  switch(ntoh16(eth_hdr->eth_type)) {
    case ETHERNET_TYPE_RUNT:
      print_runt_header((runt_hdr_t*) pkt);
      break;
    case ETHERNET_TYPE_ARP:
      print_arp_packet(pkt);
      break;
    default:
    case ETHERNET_TYPE_IPV4:
      print_eth_header((struct eth_header*)pkt);
      print_ip_header(get_ip_header(pkt));
      break;
  }
  return;
}

int send_runt_packet(uint8_t* pkt_out,
                     uint8_t* dst_mac, 
                     uint8_t* src_mac,
                     uint32_t pkt_id,
		     uint16_t pkt_type,
                     struct action_info* ai) {
  DEBUG("%s pkt_id %d\n", __func__, pkt_id);
  runt_hdr_t* runt_pkt = (runt_hdr_t*) pkt_out;
  create_eth_header(pkt_out, dst_mac, src_mac, pkt_type);
  runt_pkt->pkt_id = pkt_id;

  // INFO("print a packet before sending\n");
  // print_runt_header(runt_pkt);
  int res = nk_net_dev_send_packet(ai->netdev,
                                   pkt_out,
                                   sizeof(runt_hdr_t),
                                   TX_REQ, 0, 0);
  // DEBUG("finish sending a runt packet\n");  
  return res;
}


void delete_buf(buf_t* buffer) {
  if(!buffer->buf) return;
  free(buffer->buf);
  buffer->buf = NULL;
}

sint32_t init_buf(buf_t* buffer, uint32_t sz) {
  buffer->size = sz;
  buffer->buf = malloc(sz * sizeof(uint8_t));
  if(!buffer->buf) {
    ERROR("%s malloc cannot allocate memory\n", __func__);
    return -1;
  }

  DEBUG("%s malloc allocates at 0x%p\n", __func__, buffer->buf);
  memset(buffer->buf, 0, sz * sizeof(uint8_t));
  return 0;
}

void delete_echo_info(echo_info_t* info) {
  delete_buf(&(info->in));
  delete_buf(&(info->out));
}

sint32_t init_echo_info(echo_info_t* info,
                        struct action_info* ai,
                        uint64_t pkt_size) {
  if(!info) {
    ERROR("info = NULL\n");
    return -1;
  }

  memset(info, 0, sizeof(echo_info_t));
  
  if(!ai) {
    ERROR("ai = NULL\n");
    return -2;
  }

  info->ai = ai;
  nk_net_dev_get_characteristics(ai->netdev, &(info->dev_char));

  if(!pkt_size) {
    pkt_size = info->dev_char.max_tu;
  }
  
  uint64_t buf_size = info->dev_char.packet_size_to_buffer_size(pkt_size);
  DEBUG("buffer size %d\n", buf_size);
  init_buf(&(info->in), buf_size);
  init_buf(&(info->out), buf_size);
  if(!info->in.buf || !info->out.buf) {
    ERROR("%s cannot allocate memory\n", __func__);
    ERROR("in.buf 0x%p, out.buf 0x%p\n", info->in.buf, info->out.buf);
    return -3;
  }

  DEBUG("info->in.buf 0x%p info->out.buf 0x%p\n", 
        info->in.buf, info->out.buf);
  return 0;
}

void delete_data_item(data_item_t* item) {
  if(!item->arr) return;
  free(item->arr);
  item->arr = NULL;
}

sint32_t init_data_item(data_item_t* item, uint64_t size) {
  DEBUG("%s item 0x%p, size %lu\n", __func__, item, size);
  if(!item) {
    ERROR("item = NULL\n");
    return -1;
  }

  if(size > 0) {
    item->arr = malloc(size * sizeof(uint64_t));
    if(!item->arr) {
      ERROR("%s alloc cannot allocate data\n", __func__);
      return -2;
    }
    memset(item->arr, 0, size * sizeof(uint64_t));
  } else {
    item->arr = NULL;
    ERROR("%s size = zero\n", __func__);
  }

  DEBUG("%s item->arr = 0x%p size %lu\n", __func__, item->arr, size);
  item->total = 0;
  return 0;
}


void delete_data_collection(data_collection_t* data) {
  delete_data_item(&(data->tsc));
  delete_data_item(&(data->epkt));
  delete_data_item(&(data->rx_count));
}

void print_data_collection(data_collection_t* data) {
  INFO("data->size %lu\n", data->size);
  for(uint64_t i = 0; i < data->size; i++) {
    INFO("pkt_no |%lu| tsc |%lu| epkt |%lu|\n", i, data->tsc.arr[i], data->epkt.arr[i]);
  }
    
  // double freq = 2.2* 10e9;
  INFO("total rtt %lu\n", data->tsc.total);
  INFO("total epkt %lu\n", data->epkt.total);
  // INFO("total rx_count %lu\n", data->rx_count.total);
  // INFO("frequency %e period %e\n", freq, 1/freq);
  // INFO("average latency %e\n", data->tsc.total/(data->size + 1) * 1/freq);
}

sint32_t init_data_collection(data_collection_t *data, uint32_t data_size) {
  DEBUG("%s data = 0x%p data_size: %u\n", __func__, data, data_size);
  if(!data) { 
    ERROR("%s data = 0x%p\n", __func__);
    return -1;
  }

  if(init_data_item(&(data->tsc), data_size)) {
    ERROR("%s cannot allocate data->tsc.arr\n", __func__);
    return -2;
  }

  if(init_data_item(&(data->epkt), data_size)) {
    ERROR("%s cannot allocate data->epkt.arr\n", __func__);
    return -3;
  }

  if(init_data_item(&(data->rx_count), data_size)) {
    ERROR("%s cannot allocate data->epkt.arr\n", __func__);
    return -4;
  }
  DEBUG("%s after calling init_data_item data->size = %u\n", __func__, data_size);
  data->size = data_size;
  return 0;
}

void delete_data_op(data_op_t* dop) {
	delete_data_collection(&(dop->tx));
	delete_data_collection(&(dop->rx));
}

int init_data_op(data_op_t* dop, uint64_t data_size) {
	if(!dop) {
		ERROR("%s dop is null\n", __func__);
		return -1;
	}

	if(init_data_collection(&dop->tx, data_size)) {
    ERROR("%s cannot allocate data_op->tx\n", __func__);
    return -2;
  }
			
	if(init_data_collection(&dop->rx, data_size)) {
    ERROR("%s cannot allocate data_op->rx\n", __func__);
    return -3;
  }

  return 0;
}

void update_data_op(data_op_t* dc, volatile tsc_t* tsc_tx,
                    volatile tsc_t* tsc_rx, uint64_t index) {
  uint64_t diff = tsc_tx->end - tsc_tx->start;
  dc->tx.tsc.arr[index] = diff;
  dc->tx.tsc.total = diff;
  
  diff = tsc_rx->end - tsc_rx->start;
  dc->rx.tsc.arr[index] = diff;
  dc->rx.tsc.total = diff;
}

void print_echo_info(struct echo_info* info) {
  if(!info) {
    ERROR("info = NULL\n");
    return;
  }
  
  uint32_t size = MAC_STRING_LEN > IP_STRING_LEN ? MAC_STRING_LEN : IP_STRING_LEN;
  char buf_mac[size];
  memset(buf_mac, 0, size);

  ip_inttostr(info->ai->nk_ip_addr, buf_mac, IP_STRING_LEN);
  INFO("ai_info: nautilus machine ipv4: %s 0x%08x\n", buf_mac, info->ai->nk_ip_addr);

  mac_inttostr(info->dev_char.mac, buf_mac, MAC_STRING_LEN);
  INFO("ai_info: nautilus mac addr %s\n", buf_mac);

  INFO("ai_info: packet number: %d\n", info->ai->packet_num);
  if(info->ai->dst_ip_addr) {
    ip_inttostr(info->ai->dst_ip_addr, buf_mac, IP_STRING_LEN);
    INFO("ai_info: destination ipv4 %s\n", buf_mac); 
  } else {
    INFO("ai_info: destination ipv4 %d\n", info->ai->dst_ip_addr);
  }

  INFO("ai_info: port # %d\n", info->ai->port);
  INFO("ai_info: nic_name %s\n", info->ai->nic_name);
  return;
}
  
static void ai_thread(void *in, void **out) {
  DEBUG("%s ai_thread: in 0x%p\n", __func__, in);
  if(!in) {
    ERROR("%s in is NULL\n", __func__);
    return;
  }
  
  struct action_info* ai = (struct action_info *)in;
  echo_info_t info;
  init_echo_info(&info, ai, 0);
  print_echo_info(&info);
  
  DEBUG("ai_thread before while ------------------------------\n");
  uint64_t pkt_total_num = ai->packet_num;
  uint64_t pkt_count = 0;
  struct eth_header *eth_hdr_in = (struct eth_header*) info.in.buf;
  uint32_t runt_pkt_id = 0;
     
  while (pkt_count != pkt_total_num) {
    DEBUG("ai_thread: while receiving ------------------------------\n");
    DEBUG("pkt_count %lu pkt_id %lu\n", pkt_count, runt_pkt_id);
    nk_net_dev_receive_packet(ai->netdev, info.in.buf,
                              info.dev_char.max_tu, RX_REQ, 0, 0);

    // dump_packet(info.in.buf, sizeof(runt_hdr_t));
    if(// !compare_mac(eth_hdr_in->dst_mac, (uint8_t*) ARP_BROADCAST_MAC) &&
       !compare_mac(eth_hdr_in->dst_mac, info.dev_char.mac)) {
      continue;
    }

    DEBUG("switch cases \n");
    switch(ntoh16(eth_hdr_in->eth_type)) {
      case ETHERNET_TYPE_RUNT: {
        DEBUG("received a runt packet and print its eth header\n");
        runt_hdr_t* runt_hdr_in = (runt_hdr_t*) eth_hdr_in;
        /* print_runt_header(runt_hdr_in); */

        DEBUG("expecting packet id %d, received packet id %d\n",
              runt_pkt_id, runt_hdr_in->pkt_id);
        
        if(runt_pkt_id == runt_hdr_in->pkt_id) {
          runt_pkt_id++;
          DEBUG("match pkt_id %d\n", runt_pkt_id);
          send_runt_packet(info.out.buf, runt_hdr_in->eth_hdr.src_mac,
                           info.dev_char.mac, runt_pkt_id, ETHERNET_TYPE_RUNT,
			   ai);
          DEBUG("after sending pkt_id %d\n", runt_pkt_id);
          pkt_count++;
          runt_pkt_id++;
        } else {
          // force its sender to exit from the start runt command 
          ERROR("expected pkt_id %lu received pkt_id %lu\n", 
                runt_pkt_id, runt_hdr_in->pkt_id);
          ERROR("printing received packets before exit\n");
          send_runt_packet(info.out.buf, runt_hdr_in->eth_hdr.src_mac,
                           info.dev_char.mac, ai->packet_num, ETHERNET_TYPE_RUNT,
			   ai);
          goto EXIT_AI_THREAD;
        }

        break;
      }
	  
      case ETHERNET_TYPE_ARP: {
        struct arp_header *arp_pkt_in = get_arp_header(info.in.buf);
        if((ntoh32(arp_pkt_in->target_ip_addr) == ai->nk_ip_addr) &&
	   (ntoh16(arp_pkt_in->opcode) == ARP_OPCODE_REQUEST)) {
          print_arp_short(info.in.buf);          
          send_arp_response_packet(info.in.buf, info.out.buf, info.dev_char.mac, ai);
          DEBUG("send arp packet complete\n");
        }
        break;
      }
	
      case ETHERNET_TYPE_IPV4: {
        struct ip_header *ip_hdr_in = get_ip_header(info.in.buf);
        if(ntoh32(ip_hdr_in->ip_dst) == ai->nk_ip_addr) {

          struct icmp_header* icmp_hdr_in = get_icmp_header(info.in.buf);
          if ((ip_hdr_in->protocol == IP_PRO_ICMP) &&
              (icmp_hdr_in->type == ICMP_ECHO_REQUEST)) {
            send_icmp_packet(info.in.buf, info.out.buf, info.dev_char.mac, ai);
            
          } else if (ip_hdr_in->protocol == IP_PRO_UDP) {
            print_udp_short(info.in.buf);
            send_udp_packet(info.in.buf, info.out.buf, info.dev_char.mac, ai);
            
            if(ai->vc) {
              char buf_mac[MAC_STRING_LEN];
              char buf_ip[IP_STRING_LEN];
              nk_vc_printf("the number of UDP packets left to receive: %d\n",
			   ai->packet_num);
              mac_inttostr(eth_hdr_in->src_mac, buf_mac, MAC_STRING_LEN);
              nk_vc_printf("from MAC address: %s ", buf_mac);
              ip_inttostr(ntoh32(ip_hdr_in->ip_src), buf_ip, IP_STRING_LEN);
              nk_vc_printf("IP addres: %s\n", buf_ip);
              nk_vc_printf("UDP data: %s\n", get_udp_data(info.in.buf));
              nk_vc_printf("sending the packet back\n\n");
              DEBUG("pkt_count %d\n", pkt_count);
            }
            pkt_count++;
          }
        }
        break;
      }
    } // switch
  } // while

EXIT_AI_THREAD:
  delete_echo_info(&info);
  return;
}

void test_net_udp_echo(char* nic_name, char *ip, uint16_t port, 
                       uint32_t packet_num) {
  nk_vc_printf("Echoing %u UDP packets at address %s:%d\n", 
               packet_num,ip,port);
  struct action_info ai;
  ai.netdev = nk_net_dev_find(nic_name);
  if (!ai.netdev) {
    nk_vc_printf("Cannot find the \"%s\" from nk_net_dev\n", nic_name);
    return;
  }

  ai.nk_ip_addr = ip_strtoint(ip);
  ai.dst_ip_addr = 0;
  ai.port = port; // unused
  ai.nic_name = nic_name;
  ai.packet_num = packet_num;
  ai.vc = true;
  ai_thread((void*)&ai, NULL);
}

void test_net_send_arp_request() {
  struct action_info ai;
  ai.netdev = nk_net_dev_find("e1000e-0");
  if (!ai.netdev) {
    DEBUG("cannot find the \"%s\" from nk_net_dev\n", "e1000e-0");
    return;
  }

  ai.nk_ip_addr = ip_strtoint("165.124.183.190");
  ai.dst_ip_addr = ip_strtoint("165.124.183.169");
  nk_vc_printf("sending %d arp request back to 165.124.183.169 (r415-5)\n", 10);  
  struct nk_net_dev_characteristics c;  
  nk_net_dev_get_characteristics(ai.netdev, &c);
  uint8_t* pkt = malloc(sizeof(struct arp_packet)+80);
  if(!pkt) {
    DEBUG("cannot allocate memory space for a packet\n");
  }

  for(int i = 0; i < 10; i++) {
    nk_vc_printf("sending an arp request %d\n", i);
    send_arp_request_packet(pkt, c.mac, &ai);
    udelay(10000);
    e1000e_interpret_int_shell();   
  }
  free(pkt);
}

void test_net_send_runt(uint64_t num_runt) {
  struct action_info ai;
  ai.netdev = nk_net_dev_find("e1000e-0");

  if (!ai.netdev) {
    DEBUG("cannot find the \"%s\" from nk_net_dev\n", "e1000e-0");
    return;
  }

  ai.nk_ip_addr = ip_strtoint("165.124.183.190");
  ai.dst_ip_addr = ip_strtoint("165.124.183.169");
  nk_vc_printf("sending %d runt packets to 165.124.183.169 (r415-5)\n", num_runt);  
  struct nk_net_dev_characteristics c;  
  nk_net_dev_get_characteristics(ai.netdev, &c);
  uint8_t* pkt = malloc(sizeof(eth_hdr_t)+80);
  if(!pkt) {
    ERROR("cannot allocate memory space for a packet\n");
  }

  uint8_t dst_mac[MAC_INT_LEN];
  char* dst_mac_str ="68:05:ca_2d:a3:54";
  mac_strtoint(dst_mac, "68:05:ca_2d:a3:54");

  char mac_str[MAC_STRING_LEN];
  mac_inttostr(dst_mac, mac_str, MAC_STRING_LEN); 
  DEBUG("mac str %s mac int converted to string %s\n",
        dst_mac_str, mac_str);

  for(uint64_t i = 0; i < num_runt; i++) {
    nk_vc_printf("sending a runt packet %d\n", i);
    send_runt_packet(pkt, dst_mac, c.mac, 0, ETHERNET_TYPE_RUNT, &ai);
    udelay(10000);
    e1000e_interpret_int_shell();   
  }
  free(pkt);
}

static void start_runt(void* in, void** out) {
  DEBUG("%s function: in 0x%p\n", __func__, in);
  if(!in) {
    ERROR("%s: in is null\n", __func__);
    return;
  }

  struct action_info* ai = (struct action_info*) in;
  
  echo_info_t info;
  if(init_echo_info(&info, ai, 0)) {
    ERROR("%s: init_echo_info error\n", __func__);
    return;
  }

  INFO("print echo info\n");
  print_echo_info(&info);

  data_collection_t data_ps, data_pr, data_rtt;
  data_op_t dataop_irq, dataop_map, dataop_unmap, dataop_pkt, dataop_callback;
  data_op_t dataop_dw;
  
  DEBUG("info.ai->packet_num %u\n", ai->packet_num);
  if(init_data_collection(&data_ps, ai->packet_num)) {
    ERROR("init_data_collection error\n");
    delete_echo_info(&info);
    return;
  }

  if(init_data_collection(&data_pr, ai->packet_num)) {
    ERROR("init_data_collection error\n");
    return;
  }

  if(init_data_collection(&data_rtt, ai->packet_num)) {
    ERROR("init_data_collection error\n");
    return;
  }

  if(init_data_op(&dataop_irq, ai->packet_num)) {
    ERROR("init_data_op irq error\n");
    return;
  }

  if(init_data_op(&dataop_map, ai->packet_num)) {
    ERROR("init_data_op map error\n");
    return;
  }

  if(init_data_op(&dataop_unmap, ai->packet_num)) {
    ERROR("init_data_op unmap error\n");
    return;
  }

  if(init_data_op(&dataop_pkt, ai->packet_num)) {
    ERROR("init_data_op pkt error\n");
    return;
  }

  if(init_data_op(&dataop_callback, ai->packet_num)) {
    ERROR("init_data_op callback error\n");
    return;
  }

  if(init_data_op(&dataop_dw, ai->packet_num)) {
    ERROR("init_data_op dw error\n");
    return;
  }
  
  runt_hdr_t* runt_pkt_in = (runt_hdr_t*) info.in.buf;
  runt_hdr_t* runt_pkt_out = (runt_hdr_t*) info.out.buf;
  uint64_t pkt_total_num = info.ai->packet_num;
  
  uint32_t pkt_id = 0;
  uint64_t rtt_sta = 0;
  uint64_t rtt_end = 0;
  uint64_t rtt_total = 0;

  uint64_t data_index = 0;
  bool_t wrong_packet = false;
  
  uint64_t pkt_error = 0;
  uint64_t rx_count = 0;
  INFO("before while loop\n");

  char mac_str[MAC_STRING_LEN];
  mac_inttostr(info.dev_char.mac, mac_str, MAC_STRING_LEN); 
  DEBUG("%s device mac %s\n", __func__, mac_str);
  mac_inttostr(info.ai->dst_mac, mac_str, MAC_STRING_LEN); 
  DEBUG("%s dst mac %s\n", __func__, mac_str);
  
  while(data_index < pkt_total_num) {
    DEBUG("pkt_count %lu pkt_id %lu\n", data_index, pkt_id);

    // #measure
    rtt_sta = rdtsc(); 

    DEBUG("before send current pkt_id %lu\n", pkt_id);
    send_runt_packet(info.out.buf, info.ai->dst_mac, info.dev_char.mac,
		     pkt_id, ETHERNET_TYPE_RUNT, info.ai);
    pkt_id++;

    uint8_t recv_runt = false;
    while(!recv_runt) {
      DEBUG("after send current pkt_id %lu\n", pkt_id);
      // please do not change the type of rx_status to non-volatile
      rx_count++;
      volatile nk_net_dev_status_t rx_status = nk_net_dev_receive_packet(info.ai->netdev,
									 info.in.buf,
									 info.dev_char.max_tu,
									 RX_REQ, 0, 0);

      // INFO("after receiving a packet\n");
      // #measure
      rtt_end = rdtsc();

      if (NK_NET_DEV_STATUS_ERROR == rx_status) {
        DEBUG("receive error\n");
        print_packet(info.in.buf);
        pkt_error++;
        continue;
      }

      if(runt_pkt_in->eth_hdr.eth_type != ETHERNET_TYPE_RUNT) {
        DEBUG("wrong packet type\n");
        print_eth_header((eth_hdr_t*) info.in.buf);
        print_ip_header((ip_hdr_t*) (info.in.buf + sizeof(eth_hdr_t)));
        continue;
      }

      if(!compare_mac(runt_pkt_in->eth_hdr.dst_mac, info.dev_char.mac)) {
        wrong_packet = true;
        ERROR("fn:%s wrong mac_address type\n", __func__);
        goto EXIT_START_RUNT;
      }

      if(runt_pkt_in->pkt_id == pkt_id) {
        recv_runt = true;
        pkt_id++;
      } else {
        wrong_packet = true;
        ERROR("expected pkt_id %lu received pkt_id %lu\n", 
            pkt_id, runt_pkt_in->pkt_id);
        goto EXIT_START_RUNT;
      }
    } 

    uint64_t diff_temp = rtt_end - rtt_sta;
    data_rtt.tsc.arr[data_index] = diff_temp;
    data_rtt.tsc.total += diff_temp;
    
    data_rtt.epkt.arr[data_index] = pkt_error;
    data_rtt.epkt.total += pkt_error;
    pkt_error = 0;
    
    data_rtt.rx_count.arr[data_index] = rx_count;
    data_rtt.rx_count.total += rx_count;
    rx_count = 0;
    
    diff_temp = ps_end - ps_sta;
    data_ps.tsc.arr[data_index] = diff_temp;
    data_ps.tsc.total += diff_temp;
    data_ps.rx_count.arr[data_index] = measure.tx.dev_wait_count;
    data_ps.rx_count.total += measure.tx.dev_wait_count;
    measure.tx.dev_wait_count = 0;
    
    diff_temp = pr_end - pr_sta;
    data_pr.tsc.arr[data_index] = diff_temp;
    data_pr.tsc.total += diff_temp;
    data_pr.rx_count.arr[data_index] = measure.rx.dev_wait_count;
    data_pr.rx_count.total += measure.rx.dev_wait_count;
    measure.rx.dev_wait_count = 0;

    update_data_op(&dataop_irq, &measure.tx.irq, &measure.rx.irq, data_index);
    update_data_op(&dataop_map, &measure.tx.postx_map, &measure.rx.postx_map, data_index);
    update_data_op(&dataop_unmap, &measure.tx.irq_unmap, &measure.rx.irq_unmap, data_index);
    update_data_op(&dataop_pkt, &measure.tx.xpkt, &measure.rx.xpkt, data_index);
    update_data_op(&dataop_callback, &measure.tx.irq_callback, &measure.rx.irq_callback, data_index);
    update_data_op(&dataop_pkt, &measure.tx.xpkt, &measure.rx.xpkt, data_index);
    update_data_op(&dataop_dw, &measure.tx.dev_wait, &measure.rx.dev_wait, data_index);

#ifdef DEBUG_ECHO
    DEBUG("printing input packet\n");
    print_runt_header(runt_pkt_in);
    DEBUG("pkt_no |%lu| rtt |%lu| total_rtt |%lu|\n",
          data_index, data_rtt.tsc.arr[data_index], data_rtt.tsc.total);
    DEBUG("pkt_no |%lu| ps |%lu| total_rtt |%lu|\n",
          data_index, data_ps.tsc.arr[data_index], data_ps.tsc.total);
    DEBUG("pkt_no |%lu| pr |%lu| total_rtt |%lu|\n",
          data_index, data_pr.tsc.arr[data_index], data_pr.tsc.total);
    DEBUG("pkt_no |%lu| irq |%lu| total_rtt |%lu|\n",
          data_index, data_irq.tsc.arr[data_index], data_irq.tsc.total);
    DEBUG("%s data_index %d\n\n\n", __func__, data_index);
#endif
    data_index++;
  }
  // print_data_collection(&data);
  INFO("| 0 | 10 | 11 | 12 | 20 | 30 | 60 | 40 | 20 | 40 | 50 | 70 \n");
  INFO("| pkt# | rtt | epkt | rx_count | post_tx | tx_pkt | tx_irq | tx_map | tx_cb | tx_unmap | tx_dw | tx_dw_c\n");
  for(uint64_t i = 0; i < data_rtt.size; i++) {
    //      pkt#  rtt   epkt  rxc   ptx  txpkt txirq txmap  txcb txunm txdw  txdwc
    INFO("| %lu | %lu | %lu | %lu | %lu | %lu | %lu | %lu | %lu | %lu | %lu | %lu\n", \
         i, data_rtt.tsc.arr[i], data_rtt.epkt.arr[i], \
         data_rtt.rx_count.arr[i], data_ps.tsc.arr[i], dataop_pkt.tx.tsc.arr[i], \
         dataop_irq.tx.tsc.arr[i], dataop_map.tx.tsc.arr[i], dataop_callback.tx.tsc.arr[i], \
         dataop_unmap.tx.tsc.arr[i], dataop_dw.tx.tsc.arr[i], data_ps.rx_count.arr[i]);
  }

  INFO("\n\n\n");
  INFO("| 0 | 20 | 30 | 60 | 40 | 20 | 40 | 50 | 70 \n");
  INFO("| pkt# | post_rx | rx_pkt | rx_irq | rx_map | rx_cb | rx_unmap | rx_dw | rx_dw_c\n");
  for(uint64_t i = 0; i < data_rtt.size; i++) {
    //      pkt#  prx  rxpkt rxirq rxmap  rxcb rxunm  rxdw  rxdwc
    INFO("| %lu | %lu | %lu | %lu | %lu | %lu | %lu | %lu | %lu\n", \
         i, data_pr.tsc.arr[i], dataop_pkt.rx.tsc.arr[i], \
         dataop_irq.rx.tsc.arr[i], dataop_map.tx.tsc.arr[i], dataop_callback.tx.tsc.arr[i], \
         dataop_unmap.rx.tsc.arr[i], dataop_dw.rx.tsc.arr[i], data_pr.rx_count.arr[i]);
  }

  /* INFO("\n\n\n"); */
  /* for(uint64_t i = 0; i < data_rtt.size; i++) { */
  /*   INFO("pkt_no |%lu| set | 1 | rtt |%lu| ps |%lu| pr |%lu|\n", */
  /*        i, data_rtt.tsc.arr[i], data_ps.tsc.arr[i], data_pr.tsc.arr[i]); */
  /*   INFO("pkt_no |%lu| set | 2 | epkt |%lu| rx_count |%lu|\n", */
  /*        i, data_rtt.epkt.arr[i], data_rtt.rx_count.arr[i]); */
  /*   INFO("pkt_no |%lu| set | tx | pkt |%lu| irq |%lu| map |%lu| callback |%lu| unmap |%lu|\n", */
  /*        i, dataop_pkt.tx.tsc.arr[i], dataop_irq.tx.tsc.arr[i], dataop_map.tx.tsc.arr[i], */
  /*        dataop_callback.tx.tsc.arr[i], dataop_unmap.tx.tsc.arr[i]); */
  /*   INFO("pkt_no |%lu| set | rx | pkt |%lu| irq |%lu| map |%lu| callback |%lu| unmap |%lu|\n", */
  /*        i, dataop_pkt.rx.tsc.arr[i], dataop_irq.rx.tsc.arr[i], dataop_map.rx.tsc.arr[i], */
  /*        dataop_callback.rx.tsc.arr[i], dataop_unmap.rx.tsc.arr[i]); */
  /*   INFO("pkt_no |%lu| set | dw | tx |%lu| rx |%lu|\n", */
  /*        i, dataop_dw.tx.tsc.arr[i], dataop_dw.rx.tsc.arr[i]); */
  /* } */

  INFO("total rtt %lu\n", data_rtt.tsc.total);
  INFO("total rtt ps %lu\n", data_ps.tsc.total);
  INFO("total rtt pr %lu\n", data_pr.tsc.total);
  INFO("total epkt %lu\n", data_rtt.epkt.total);
  INFO("total rx_count %lu\n", data_rtt.rx_count.total);
  INFO("total tx dw count %lu\n", data_ps.rx_count.total);
  INFO("total rx dw count %lu\n", data_pr.rx_count.total);

EXIT_START_RUNT:
  if(wrong_packet) {
    ERROR("fn:%s printing received packets before exit\n", __func__);
    ERROR("fn:%s pkt_count %lu\n", __func__, data_index);
    print_runt_header(runt_pkt_in);
    dump_packet((uint8_t*) runt_pkt_in, sizeof(runt_hdr_t));
    send_runt_packet(info.out.buf, info.ai->dst_mac, info.dev_char.mac,
		     info.ai->packet_num + 1, ETHERNET_TYPE_IRUNT, info.ai); 
  }

  delete_echo_info(&info);
  delete_data_collection(&data_rtt);
  delete_data_collection(&data_ps);
  delete_data_collection(&data_pr);

  delete_data_op(&dataop_irq);
  delete_data_op(&dataop_map);
  delete_data_op(&dataop_unmap);
  delete_data_op(&dataop_pkt);
  delete_data_op(&dataop_callback);
  delete_data_op(&dataop_dw);
  
  return;
}

static int select_dst_mac(uint32_t machine_no, uint8_t* dst_mac) {
  char* dst_mac_char = NULL;
  switch(machine_no) {
    case MACHINE_NO_R4154:  
      INFO("Select R415-4 Mac Address\n"); 
      dst_mac_char = MAC_R4154_E1000E;
      break;
    case MACHINE_NO_R4155:
      INFO("Select R415-5 Mac Address\n"); 
      dst_mac_char = MAC_R4155_E1000E;
      break;
    case MACHINE_NO_R4156:
      INFO("Select R415-6 Mac Address\n"); 
      dst_mac_char = MAC_R4156_E1000E;
      break;
    default:
      ERROR("unknown machine number %d\n", machine_no);
      return -1;
  }
  INFO("dst_mac %s\n", dst_mac_char);
  mac_strtoint(dst_mac, dst_mac_char);
  char buf[MAC_STRING_LEN];
  mac_inttostr(dst_mac, buf, sizeof(buf));
  INFO("dst_mac (debug): %s\n", buf);
  return 0;
}

static int setup_info(struct action_info* ai,
		      char* nic_name,
		      uint32_t machine_no,
		      uint32_t packet_num,
		      uint32_t optimize) {
  if(!ai) {
    ERROR("ai is null\n");
    return -1;
  }

  if(optimize) {
    e1000e_opt_shell();
    INFO("optimize nic\n");
  } else {
    e1000e_no_opt_shell();
    INFO("no optimize nic\n");
  }

  ai->netdev = nk_net_dev_find(nic_name);
  if (!ai->netdev) {
    nk_vc_printf("Cannot find the \"%s\" from nk_net_dev\n", nic_name);
    return -2;
  }
  
  ai->nk_ip_addr = 0;
  ai->dst_ip_addr = 0;
  
  if(select_dst_mac(machine_no, ai->dst_mac)) {
    ERROR("select_dst_mac machine_no %d\n", machine_no);
    return -3;
  }

  ai->port = 0; // unused
  ai->nic_name = nic_name;
  ai->packet_num = packet_num;
  ai->vc = true;
  return 0;
}

void test_net_start_runt(char* nic_name,
                         uint32_t machine_no,
                         uint32_t packet_num,
                         bool_t optimize) {
  INFO("Starting %u runt packets at %s devices to this %d machine optimize %u\n", 
       packet_num, nic_name, machine_no, optimize);

  struct action_info ai;
  if(setup_info(&ai, nic_name, machine_no, packet_num, optimize)) {
    ERROR("cannot setup an info, and exit from this command\n");
    return;
  }

  INFO("calling start runt echo\n");
  start_runt((void*)&ai, NULL);
  INFO("returning to the shell\n");
  return;
}

static int echo_runt(void* in, void** out) {
  DEBUG("%s: in 0x%p\n", __func__, in);
  if(!in) {
    ERROR("%s: in is null\n", __func__);
    return -1;
  }

  struct action_info* ai = (struct action_info*) in;

  echo_info_t info;
  if(init_echo_info(&info, ai, 0)) {
    ERROR("%s: init_echo_info error\n", __func__);
    return -2;
  }

  INFO("print echo info\n");
  print_echo_info(&info);

  runt_hdr_t* runt_pkt_in = (runt_hdr_t*) info.in.buf;
  uint64_t pkt_total_num = info.ai->packet_num;
  uint32_t pkt_id = 0;
  uint32_t pkt_count = 0;
  bool_t wrong_packet = false;
  while(pkt_count < pkt_total_num) {
    DEBUG("pkt_count %lu pkt_id %lu\n", pkt_count, pkt_id);
    uint8_t recv_runt = false;
    while(!recv_runt) {
      DEBUG("receiving a packet\n");
      // please do not change the type of rx_status to non-volatile
      volatile nk_net_dev_status_t rx_status = nk_net_dev_receive_packet(info.ai->netdev,
									 info.in.buf,
									 info.dev_char.max_tu,
									 RX_REQ, 0, 0);

      if (NK_NET_DEV_STATUS_ERROR == rx_status) {
	DEBUG("packet error\n");
	print_packet(info.in.buf);
	continue;
      }

      // INFO("%s after receiving a runt packet expected pkt_id %lu\n", __func__, pkt_id);
      // print_runt_header((runt_hdr_t*) info.in.buf); 
      
      if(runt_pkt_in->eth_hdr.eth_type != ETHERNET_TYPE_RUNT) {
	DEBUG("wrong packet type\n");
	print_packet(info.in.buf);
	continue;
      }

      if(!compare_mac(runt_pkt_in->eth_hdr.dst_mac, info.dev_char.mac)) {
	wrong_packet = true;
	ERROR("fn:%s wrong mac_address type\n", __func__);
	goto EXIT_ECHO_RUNT;
      }
      
      if(runt_pkt_in->pkt_id == pkt_id) {
	pkt_id++;
        recv_runt = true;
      } else {
	wrong_packet = true;
        ERROR("fn:%s expected pkt_id %lu received pkt_id %lu\n",
	      __func__, pkt_id, runt_pkt_in->pkt_id);
        goto EXIT_ECHO_RUNT;
      }
    } 

    send_runt_packet(info.out.buf, runt_pkt_in->eth_hdr.src_mac,
		     info.dev_char.mac, pkt_id, ETHERNET_TYPE_RUNT, ai);
    DEBUG("after sending a runt packet pkt_id %d\n\n\n", pkt_id);
    pkt_count++;
    pkt_id++;
  }

EXIT_ECHO_RUNT:
  if(wrong_packet) {
    ERROR("printing received packets before exit\n");
    ERROR("pkt_count %lu\n", pkt_count);
    print_runt_header(runt_pkt_in);
    dump_packet((uint8_t*) runt_pkt_in, sizeof(runt_hdr_t));
    /* send_runt_packet(info.out.buf, info.ai->dst_mac, info.dev_char.mac, */
    /* 		     info.ai->packet_num + 1, ETHERNET_TYPE_RUNT, info.ai); */
  }
    
  delete_echo_info(&info);
  return 0;
}

void test_net_echo_runt(char* nic_name,
			uint32_t machine_no,
			uint32_t packet_num,
			bool_t optimize) {
  INFO("Starting %u runt packets at %s devices to the %d machine optimize %u\n", 
       packet_num, nic_name, machine_no, optimize);

  struct action_info ai;
  if(setup_info(&ai, nic_name, machine_no, packet_num, optimize)) {
    ERROR("cannot setup an info, and exit from this command\n");
    return;
  }

  INFO("calling echo_runt function\n");
  echo_runt((void*)&ai, NULL);
  INFO("%s returning to the shell\n", __func__);
  return;
}

