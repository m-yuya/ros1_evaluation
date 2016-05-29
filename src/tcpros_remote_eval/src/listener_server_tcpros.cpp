#include "ros/ros.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>				// clock
#include <unistd.h>				// clock
#include <sys/mman.h>			// mlock
#include <sched.h>				// sched
#include <arpa/inet.h>			// socket

#define EVAL_NUM 120

int i, initializer=0;

struct timespec tp1;			// for clock

FILE *fp;						// for file io

struct sockaddr_in addr;		// for socket
struct sockaddr_in client;
socklen_t client_size;
int sock0, sock;

void chatterCallback(const std_msgs::String::ConstPtr& msg){

  //  printf("=============================\n");
  //  printf("subscribe: [%s]\n", receiver.data.c_str());
  //  printf("subscribe \n");

  if( initializer == 0 ) {
	// Initialize
	char init_num_char = *( msg->data.c_str());
	char *init_num_pt = &init_num_char;
	initializer = atoi(init_num_pt);
	//	printf("initializer : %d \n", initializer);
	if ( initializer == 1 ){
	  printf("start evaluation as a server \n");
	}
  }
  if ( initializer == 1 ){
	/* write(ソケット,"文字",文字数) */
	//printf("write \n");
	write(sock, "x", 1);
  }
}
int set_bind_listen_accept_socket(){

  /* ソケットの作成 */
  printf("set \n");
  sock0 = socket(AF_INET, SOCK_STREAM, 0);
  if( sock0<0 ){
  	perror("socket");
  	return 1;
  }

  /* ソケットの設定 */
  addr.sin_family = AF_INET;
  addr.sin_port = htons(12345);
  addr.sin_addr.s_addr = INADDR_ANY;

  // portがTIME_WAIT状態でも接続する
  const int one = 1;
  setsockopt(sock0, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(int));

  printf("bind \n");
  if (  bind(sock0, (struct sockaddr *)&addr, sizeof(addr))<0 ){
  	perror("bind");
  	return 1;
  }

  /* TCPクライアントからの接続要求を待てる状態にする */
  printf("listen \n");
  if(   listen(sock0, 5)<0 ){
  	perror("listen");
  	return 1;
  }

  client_size = sizeof(client);
  printf("accept \n");
  printf("waiting for talker_client... \n");

  //クライアントから通信があるまで待機
  sock = accept(sock0, (struct sockaddr *)&client, &client_size);
  if( sock<0 ){
	perror("accept");
	return 1;
  }

  printf("accepted connection from %s, port=%d\n",
		 inet_ntoa(client.sin_addr), ntohs(client.sin_port));

  return 0;
}

int main(int argc, char **argv)
{
  mlockall(MCL_FUTURE);
  
  usleep(1000);					// avoid race condition
  sched_param  pri = {94}; 
  if (sched_setscheduler(0, SCHED_FIFO, &pri) == -1) {
	perror("sched_setattr");
	exit(EXIT_FAILURE);
  }
  
  ros::init(argc, argv, "listener");
  
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, +chatterCallback);

  // wait for establishing socket
  if ( set_bind_listen_accept_socket() == 1 ){
  	perror("set_bind_listen_accept_socket");
  	return 1;
  }

  ros::spin();

  return 0;
}
