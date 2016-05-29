#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/String.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>				// clock
#include <unistd.h>				// clock
#include <sys/mman.h>			// mlock
#include <sched.h>				// sched

#define EVAL_NUM 120


struct timespec tp2;
int j, count2 = -1, init_num_int;
double  subscribe_time[EVAL_NUM];

std_msgs::String receiver;

FILE *fp2;

int eval_loop_count = 0;

std::string output_filename = "./evaluation/subscribe_time/subscribe_time_256byte.txt";

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{

  if( count2 == -1 ){

	// Initialize count2
	char init_num_char = *( msg->data.c_str());
	char *init_num_pt = &init_num_char;
	count2 = atoi(init_num_pt);
	init_num_int = count2;    

	// printf("first recieved number: %d \n\n", count2);
	printf("message loss : %d \n", init_num_int);
   
  }
  
  // evaluation
  if( count2 < EVAL_NUM-1 ){

	receiver = *msg;
	if(clock_gettime(CLOCK_REALTIME,&tp2) < 0){
	  perror("clock_gettime begin");
	}
	subscribe_time[count2] = (double)tp2.tv_sec + (double)tp2.tv_nsec/ (double)1000000000L;

	// printf("%18.9lf\n",subscribe_time[count2]);
	//	printf("subscribe_time[%2d]:\t%18.9lf\n", count2,subscribe_time[count2]);

	// printf("I heard: [%s]\n", receiver.data.c_str());
	// printf("I heard: [%c]\n",* ( msg->data.c_str()) );

	// printf("Time Span:\t%ld.%09ld\n", tp2.tv_sec, tp2.tv_nsec);

	count2++;

  }else if( count2 == EVAL_NUM-1 ){
    
    receiver = *msg;
	if(clock_gettime(CLOCK_REALTIME,&tp2) < 0){
	  perror("clock_gettime begin");
	}
	subscribe_time[count2] = (double)tp2.tv_sec + (double)tp2.tv_nsec/ (double)1000000000L;

	// 評価終了後にまとめてpublish_time[]をpublish_time.txtに出力
	if((fp2 = fopen(output_filename.c_str(), "w")) != NULL){
	
	  // init_numの書き込み
	  if(fprintf(fp2, "%d\n",init_num_int ) < 0){
		//書き込みエラー
		printf("error : can't output subscribe_time.txt'");
	  }

	  // subscribe_time[]の書き込み
	  for(j=0; j<EVAL_NUM; j++){
		if(fprintf(fp2, "%18.9lf\n", subscribe_time[j]) < 0){
		  //書き込みエラー
		  printf("error : can't output subscribe_time.txt'");
		  break;
		}
	  }

	  // printf("output data\n");

	  fclose(fp2);
	}else{
	  printf("error : can't output subscribe_time.txt'");
	}

	// 評価の初期化
	count2 = -1;
	eval_loop_count++;

	if( eval_loop_count == 1){
	  output_filename = "./evaluation/subscribe_time/subscribe_time_512byte.txt";
	}else if( eval_loop_count == 2){
	  output_filename = "./evaluation/subscribe_time/subscribe_time_1Kbyte.txt";
	}else if( eval_loop_count == 3){
	  output_filename = "./evaluation/subscribe_time/subscribe_time_2Kbyte.txt";
	}else if( eval_loop_count == 4){
	  output_filename = "./evaluation/subscribe_time/subscribe_time_4Kbyte.txt";
	}else if( eval_loop_count == 5){
	  output_filename = "./evaluation/subscribe_time/subscribe_time_8Kbyte.txt";
	}else if( eval_loop_count == 6){
	  output_filename = "./evaluation/subscribe_time/subscribe_time_16Kbyte.txt";
	}else if( eval_loop_count == 7){
	  output_filename = "./evaluation/subscribe_time/subscribe_time_32Kbyte.txt";
	}else if( eval_loop_count == 8){
	  output_filename = "./evaluation/subscribe_time/subscribe_time_64Kbyte.txt";
	}else if( eval_loop_count == 9){
	  output_filename = "./evaluation/subscribe_time/subscribe_time_128Kbyte.txt";
	}else if( eval_loop_count == 10){
	  output_filename = "./evaluation/subscribe_time/subscribe_time_256Kbyte.txt";
	}else if( eval_loop_count == 11){
	  output_filename = "./evaluation/subscribe_time/subscribe_time_512Kbyte.txt";
	}else if( eval_loop_count == 12){
	  output_filename = "./evaluation/subscribe_time/subscribe_time_1Mbyte.txt";
	}else if( eval_loop_count == 13){
	  output_filename = "./evaluation/subscribe_time/subscribe_time_2Mbyte.txt";
	}else if( eval_loop_count == 14){
	  output_filename = "./evaluation/subscribe_time/subscribe_time_4Mbyte.txt";
	}else if( eval_loop_count == 13){
	  // 計測終了
	  count2 == EVAL_NUM;
	}
	
  }
}

namespace nodelet_eval
{

class Listener : public nodelet::Nodelet
{
public:
  Listener() {}

private:
  virtual void onInit()
  {
	  
	mlockall(MCL_FUTURE);
  
	usleep(1000);
	sched_param  pri = {94}; 
	if (sched_setscheduler(0, SCHED_FIFO, &pri) == -1) {
	  perror("sched_setattr");
	  exit(EXIT_FAILURE);
	}

    ros::NodeHandle& private_nh = getPrivateNodeHandle();

	// sub = private_nh.subscribe("/out", 10, &Listener::callback, this);
    sub = private_nh.subscribe("/chatter", 10, +chatterCallback);

	printf("Launched nodelet_eval/Listener \n");

  }
  
  // void callback(const std_msgs::String::ConstPtr& input)
  // {
  // ROS_INFO("%s", input->data.c_str());
  //	printf("listener : %p \n", input.get());
	
  // }

  ros::Subscriber sub;
};

PLUGINLIB_DECLARE_CLASS(nodelet_eval, Listener, nodelet_eval::Listener, nodelet::Nodelet);
}
