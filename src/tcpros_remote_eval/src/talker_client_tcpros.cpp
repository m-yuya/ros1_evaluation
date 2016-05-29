#include <ros/ros.h>
#include <std_msgs/String.h>

#include <stdio.h>
#include <sstream>
#include <fstream>
#include <string>
#include <iostream>				// file io
#include <time.h>				// clock
#include <unistd.h>				// clock
#include <sys/mman.h>			// mlock
#include <sched.h>				// sched
#include <arpa/inet.h>			// socket

#define EVAL_NUM 120
#define LISTENER_IP "192.168.1.70"		
#define PUBLISH_Hz 10

int i, count = -1;
double publish_time[EVAL_NUM];
double subscribe_time[EVAL_NUM];
double transport_time[EVAL_NUM];

std::string s, bytedata;

struct timespec tp1;			// for clock

FILE *fp;						// for file io

struct sockaddr_in server;		// for socket
int sock;
char buf[1];
int len;

// file名を引数に取りフアイルの中身を返す関数
std::string read_datafile(std::string message_filename){
  // data_*byte.txtからstd::string bytedataへデータを読み込み 
  
  std::ifstream ifs(message_filename.c_str());
  if(ifs.fail()) {
	std::cerr << "data_*byte.txt do not exist.\n";
	exit(0);
  }

  std::string bytedata;
  getline(ifs, bytedata);

  return bytedata;
}

// EVAL_NUM回、message_filename(data_*byte.txt)をpublishして時刻をoutput_filename(transport_time_*byte.txt)へ出力
int eval_remote_client_ros1(std::string message_filename, std::string output_filename, ros::Publisher chatter_pub){
  if( -1 < count ) {

	std_msgs::String msg;
	std::stringstream ss;
  
	// messageの作成
	ss << 1;					// prefix 1 : listener calls "write()"
	s = ss.str() + bytedata;
	msg.data = s;
	
	// 時刻の記録
	if(clock_gettime(CLOCK_REALTIME,&tp1) < 0){
	  perror("clock_gettime begin");
	  return 0;
	}
	publish_time[count] = (double)tp1.tv_sec + (double)tp1.tv_nsec/ (double)1000000000L;
	  
	// printf("publish: [%s] at %lf\n", msg.data.c_str(), publish_time[count]);

	//	printf("publish \n");
	chatter_pub.publish(msg);

	/* サーバからデータを受信 */
	//	printf("read \n");
	len = read(sock, buf, sizeof(buf));
	if( len <= 0 ){
	  perror("read");
	  return 1;
	}
	//	printf("%d, %s\n", len, buf);
	
	if(clock_gettime(CLOCK_REALTIME,&tp1) < 0){
	  perror("clock_gettime begin");
	}
	subscribe_time[count] = (double)tp1.tv_sec + (double)tp1.tv_nsec/ (double)1000000000L;
  
  }
  else if(count == -1){
	
	bytedata = read_datafile(message_filename.c_str());
	
  }

  // 通信時間を計算し、transport_time.txtへ出力  
  if( count == EVAL_NUM - 1){

	if((fp = fopen(output_filename.c_str(), "w")) != NULL){
	  for(i=0; i<=count; i++){
		transport_time[i] = subscribe_time[i] - publish_time[i];
		if(fprintf(fp, "%1.9lf\n", transport_time[i]) < 0){
		  //書き込みエラー
		  break;
		}
	  }
	  fclose(fp);
	}else{
	  printf("error : can't output file \n");
	}

	count = -2;					// initialize for next data size 
	
  }
  
  count++;
  
  return 0;

}

int set_connect_socket(std::string IP_ADDRESS){

  /* ソケットの作成 */
  printf("set \n");
  sock = socket(AF_INET, SOCK_STREAM, 0);
  if( sock<0 ){
	perror("socket");
	return 1;
  }

  /* 接続先指定用構造体の準備 */
  server.sin_family = AF_INET;
  server.sin_port = htons(12345);
  server.sin_addr.s_addr = inet_addr(IP_ADDRESS.c_str()); // IPアドレスを設定

  /* サーバに接続 */
  printf("connect \n");
  if( connect(sock, (struct sockaddr *)&server, sizeof(server))<0 ){
	perror("connect");
	return 0;
  }

  memset(buf, 0, sizeof(buf));

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

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  
  ros::Rate loop_rate(PUBLISH_Hz);

  // request establishing socket to IP
  if( set_connect_socket(LISTENER_IP) == 1 ){
	perror("set_connect_socket");
	return 1;
  }

  // meanless 20 publish for preparing a secure subscriber
  // 5 and 10 is not enough to conver messages ros1 <-> ros2
  std_msgs::String msg;
  std::stringstream ss;
  ss << 0;						// prefix 0 : listener does not call "write()"
  s = ss.str() + "start";
  msg.data = s;
  i = 1;
  while (ros::ok()) {
	if( i++ > 100 ){
	  break;
	}
	//	printf("publish \n");
	chatter_pub.publish(msg);
	ros::spinOnce();
	loop_rate.sleep();
  }

  printf("start evaluation 256byte \n");
  while (ros::ok()) {
	eval_remote_client_ros1("./evaluation/byte_data/data_256byte.txt", "./evaluation/transport_time/transport_time_256byte.txt", chatter_pub);
	if(count == -1){
	  printf("break\n");
	  break;
	}
    ros::spinOnce();
    loop_rate.sleep();
  }

  usleep(1000000);				// avoid race condition
  
  printf("start evaluation 512byte \n");
  while (ros::ok()) {
	eval_remote_client_ros1("./evaluation/byte_data/data_512byte.txt", "./evaluation/transport_time/transport_time_512byte.txt", chatter_pub);
	if(count == -1){
	  printf("break\n");
	  break;
	}
    ros::spinOnce();
    loop_rate.sleep();
  }

  usleep(1000000);

  printf("start evaluation 1Kbyte \n");
while (ros::ok()) {
	eval_remote_client_ros1("./evaluation/byte_data/data_1Kbyte.txt", "./evaluation/transport_time/transport_time_1Kbyte.txt", chatter_pub);
	if(count == -1){
	  printf("break\n");
	  break;
	}
    ros::spinOnce();
    loop_rate.sleep();
  }

  usleep(1000000);

  printf("start evaluation 2Kbyte \n");
  while (ros::ok()) {
	eval_remote_client_ros1("./evaluation/byte_data/data_2Kbyte.txt", "./evaluation/transport_time/transport_time_2Kbyte.txt", chatter_pub);
	if(count == -1){
	  printf("break\n");
	  break;
	}
    ros::spinOnce();
    loop_rate.sleep();
  }

  usleep(1000000);

  printf("start evaluation 4Kbyte \n");
  while (ros::ok()) {
	eval_remote_client_ros1("./evaluation/byte_data/data_4Kbyte.txt", "./evaluation/transport_time/transport_time_4Kbyte.txt", chatter_pub);
	if(count == -1){
	  printf("break\n");
	  break;
	}
    ros::spinOnce();
    loop_rate.sleep();
  }

  usleep(1000000);

  printf("start evaluation 8Kbyte \n");
  while (ros::ok()) {
	eval_remote_client_ros1("./evaluation/byte_data/data_8Kbyte.txt", "./evaluation/transport_time/transport_time_8Kbyte.txt", chatter_pub);
	if(count == -1){
	  printf("break\n");
	  break;
	}
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  usleep(1000000);
  
  printf("start evaluation 16Kbyte \n");
  while (ros::ok()) {
	eval_remote_client_ros1("./evaluation/byte_data/data_16Kbyte.txt", "./evaluation/transport_time/transport_time_16Kbyte.txt", chatter_pub);
	if(count == -1){
	  printf("break\n");
	  break;
	}
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  usleep(1000000);

  printf("start evaluation 32Kbyte \n");
  while (ros::ok()) {
	eval_remote_client_ros1("./evaluation/byte_data/data_32Kbyte.txt", "./evaluation/transport_time/transport_time_32Kbyte.txt", chatter_pub);
	if(count == -1){
	  printf("break\n");
	  break;
	}
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  usleep(1000000);
  
  printf("start evaluation 64Kbyte \n");
  while (ros::ok()) {
	eval_remote_client_ros1("./evaluation/byte_data/data_64Kbyte.txt", "./evaluation/transport_time/transport_time_64Kbyte.txt", chatter_pub);
	if(count == -1){
	  printf("break\n");
	  break;
	}
    ros::spinOnce();
    loop_rate.sleep();
  }

  usleep(1000000);

  printf("start evaluation 128Kbyte \n");
  while (ros::ok()) {
	eval_remote_client_ros1("./evaluation/byte_data/data_128Kbyte.txt", "./evaluation/transport_time/transport_time_128Kbyte.txt", chatter_pub);
	if(count == -1){
	  printf("break\n");
	  break;
	}
    ros::spinOnce();
    loop_rate.sleep();
  }

  usleep(1000000);

  printf("start evaluation 256Kbyte \n");
  while (ros::ok()) {
	eval_remote_client_ros1("./evaluation/byte_data/data_256Kbyte.txt", "./evaluation/transport_time/transport_time_256Kbyte.txt", chatter_pub);
	if(count == -1){
	  printf("break\n");
	  break;
	}
    ros::spinOnce();
    loop_rate.sleep();
  }

  usleep(1000000);
  
  printf("start evaluation 512Kbyte \n");
  while (ros::ok()) {
	eval_remote_client_ros1("./evaluation/byte_data/data_512Kbyte.txt", "./evaluation/transport_time/transport_time_512Kbyte.txt", chatter_pub);
	if(count == -1){
	  printf("break\n");
	  break;
	}
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  usleep(1000000);
  
  printf("start evaluation 1Mbyte \n");
  while (ros::ok()) {
	eval_remote_client_ros1("./evaluation/byte_data/data_1Mbyte.txt", "./evaluation/transport_time/transport_time_1Mbyte.txt", chatter_pub);
	if(count == -1){
	  printf("break\n");
	  break;
	}
    ros::spinOnce();
    loop_rate.sleep();
  }

  usleep(1000000);
  
  printf("start evaluation 2Mbyte \n");
  while (ros::ok()) {
	eval_remote_client_ros1("./evaluation/byte_data/data_2Mbyte.txt", "./evaluation/transport_time/transport_time_2Mbyte.txt", chatter_pub);
	if(count == -1){
	  printf("break\n");
	  break;
	}
    ros::spinOnce();
    loop_rate.sleep();
  }

  usleep(1000000);
  
  printf("start evaluation 4Mbyte \n");
  while (ros::ok()) {
	eval_remote_client_ros1("./evaluation/byte_data/data_4Mbyte.txt", "./evaluation/transport_time/transport_time_4Mbyte.txt", chatter_pub);
	if(count == -1){
	  printf("break\n");
	  break;
	}
    ros::spinOnce();
    loop_rate.sleep();
  }

  // followthrough transactions
  msg.data = "end";
  i = 1;
  while (ros::ok()) {
	chatter_pub.publish(msg);
	if( i++ > 5){
	  break;
	}
    ros::spinOnce();
    loop_rate.sleep();
  }

  /* socketの終了 */
  close(sock);

  printf("---end evaluation---\n");

  return 0;
}
