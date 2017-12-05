#ifndef _UDP_LINK_H_
#define _UDP_LINK_H_

#define MAX_MSG_LEN	512
#define UDP_LINK_DEBUG	0

struct udp_node{
	long ts;
	long tr;
	char* msg;
	struct udp_node *next;
};


//int check_if_len_error(int link_len, int msg_len);
//struct udp_node *init_node(int id, int len);
//void print_link(struct udp_node *link);
struct udp_node *init_link(int link_len);
void free_link(struct udp_node *link);

#endif
