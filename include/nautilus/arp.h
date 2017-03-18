#ifndef __ARP
#define __ARP

#define IPV4_LEN                  4              /* length of ipv4 in bytes */
#define MAC_LEN                   6              /* length of mac address in bytes */
#define ARP_ETHERNET_HW_TYPE      1
#define ARP_IP_PRO_TYPE           0x800
#define ARP_REQUEST_OPCODE        0x0001
#define ARP_REPLY_OPCODE          0x0002d
#define IP_ADDRESS_STRING         "10.10.10.3"

extern const uint8_t ARP_BROADCAST_MAC[6];

struct arp_packet {
  volatile uint16_t hw_type;                    /* hardware address */
  volatile uint16_t pro_type;                   /* protocol address */
  volatile uint8_t hw_len;                      /* hardware address length */
  volatile uint8_t pro_len;                     /* protocol address length */
  volatile uint16_t opcode;                     /* arp opcode */
  volatile uint8_t sender_hw_addr[MAC_LEN];     /* sender hardware address */
  volatile uint32_t sender_ip_addr;             /* sender protocol address */
  volatile uint8_t target_hw_addr[MAC_LEN];     /* target hardware address */
  volatile uint32_t target_ip_addr;             /* target protocol address */
} __attribute__((packed));

struct arp_info {
  struct nk_net_dev *netdev;
  uint32_t ip_addr;
};

int arp_init(struct naut_info *);
int arp_deinit();

#endif

