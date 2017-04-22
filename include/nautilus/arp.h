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

// ARP frame
#define ARP_HW_TYPE_ETHERNET      1              /* arp hardware type for ethernet */
#define ARP_PRO_TYPE_IPV4         0x0800         /* arp protocol type for ipv4 */
#define ARP_OPCODE_REQUEST        0x0001         /* arp opcode request */
#define ARP_OPCODE_REPLY          0x0002         /* arp opcode reply */

// ethernet frame
#define ETHERNET_TYPE_ARP         0x0806         /* ethernet type arp */
#define ETHERNET_TYPE_IPV4        0x0800         /* ethernet type ipv4 */
#define ETHERNET_TYPE_IPX         0x8137         /* ethernet type ipx */
#define ETHERNET_TYPE_IPV6        0x86dd         /* ethernet type ipv6 */

extern const uint8_t ARP_BROADCAST_MAC[6];

// ethernet header
struct eth_header {
  uint8_t  dst_mac[MAC_LEN];
  uint8_t  src_mac[MAC_LEN];
  uint16_t eth_type;
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

// function definition
void dump_packet(uint8_t *p, int len);
int arp_init(struct naut_info *);
int arp_deinit();

#endif

