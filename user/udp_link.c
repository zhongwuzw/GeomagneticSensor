#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "los_dev_st_uart.h"
#define MAX_MSG_LEN	64
#define UDP_LINK_DEBUG	0

struct udp_node{
	long ts;
	long tr;
	struct udp_node *next;
};


int check_if_len_error(int link_len, int msg_len)
{
	int i   = 0;
	int len = link_len;

	while(len){
		len /= 10;
		i ++;
	}

	if(i > msg_len || msg_len > MAX_MSG_LEN){
		return 1;
	}

	return 0;
}
struct udp_node *init_node()
{
	struct udp_node *node = (struct udp_node *)malloc(sizeof(struct udp_node));

	if(!node){
		return NULL;
	}

	memset(node, 0, sizeof(struct udp_node));
	//snprintf(node->msg, MAX_MSG_LEN, "%0*d", len, id);
	node->next = NULL;

	return node;
}


struct udp_node *init_link(int link_len)
{
	int i;
	struct udp_node *node;
	struct udp_node *head = NULL;

	for(i = 0; i < link_len; i ++){
		node = init_node();
		memset(node,0,sizeof(struct udp_node));
		if(head){
			node->next = head;
		}
		head = node;	
	}
	
	return head;
}



void free_link(struct udp_node *link){
	struct udp_node *node;

	for(; link;){
		node = link;
		link = link->next;
		free(node);
	}
}


#if UDP_LINK_DEBUG
int main()
{
	struct udp_node *link = NULL;

	link = init_link(10, 10);
	if(!link){
		printf("Link = NULL\n");
		return -1;
	}
	print_link(link);
	free_link(link);
	return 0;	
}
#endif
