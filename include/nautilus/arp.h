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

#ifndef __ARP
#define __ARP

// general definition
#define IPV4_LEN                  4              /* length of ipv4 in bytes */
#define MAC_LEN                   6              /* length of mac address in bytes */
#define IP_ADDRESS_STRING         "10.10.10.3"

// ethernet frame
#define ETHERNET_TYPE_ARP         0x0806         /* ethernet type arp */
#define ETHERNET_TYPE_IPV4        0x0800         /* ethernet type ipv4 */
#define ETHERNET_TYPE_IPX         0x8137         /* ethernet type ipx */
#define ETHERNET_TYPE_IPV6        0x86dd         /* ethernet type ipv6 */

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

extern const uint8_t ARP_BROADCAST_MAC[6];

// ethernet header
// size = 18 bytes
struct eth_header {
  uint8_t  dst_mac[MAC_LEN];        /* destination mac address */
  uint8_t  src_mac[MAC_LEN];        /* source mac address */
  uint16_t eth_type;                /* ethernet type */
} __packed;

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

// udp header
//  0              15  16             31
//  +--------+--------+--------+--------+
//  |     Source      |   Destination   |
//  |      Port       |      Port       |
//  +--------+--------+--------+--------+
//  |    Checksum     |                 |
//  |    Coverage     |    Checksum     |
//  +--------+--------+--------+--------+
//  |                                   |
//  :              Payload              :
//  |                                   |
//  +-----------------------------------+
// size 8 bytes
struct udp_header {
  uint16_t src_port;                   /* source port */
  uint16_t dst_port;                   /* destination port */
  uint16_t length;                     /* length */
  uint16_t checksum;                   /* checksum */
} __packed;

// Internet Protocol (IPv4) over Ethernet ARP packet
struct arp_packet {
  uint16_t hw_type;                    /* hardware address */
  uint16_t pro_type;                   /* protocol address */
  uint8_t hw_len;                      /* hardware address length */
  uint8_t pro_len;                     /* protocol address length */
  uint16_t opcode;                     /* arp operation */
  uint8_t sender_mac[MAC_LEN];         /* sender hardware address */
  uint32_t sender_ip_addr;             /* sender protocol address */
  uint8_t target_mac[MAC_LEN];         /* target hardware address */
  uint32_t target_ip_addr;             /* target protocol address */
} __attribute__((packed));

struct arp_info {
  struct nk_net_dev *netdev;
  uint32_t ip_addr;
};

// icmp header
// size = 8 without data
struct icmp_header {
  uint8_t type;                        /* type */
  uint8_t code;                        /* code */
  uint16_t checksum;                   /* icmp header checksum */
  uint16_t id;                         /* identifier */
  uint16_t seq_num;                    /* sequence number */
} __packed;

// function definition
void dump_packet(uint8_t *p, int len);
int arp_init(struct naut_info *);
int arp_deinit();

#endif
