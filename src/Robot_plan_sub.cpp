#include <stdio.h>
#include <string.h>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "verbal_navigation/Wavenet.h"
#include <sound_play/sound_play.h>
#include <unistd.h>

void planCallback(){
}
int main(int argc, char **argv){
   ros::Subscriber plan_sub = n.subscribe("Plan", 1000, 
}
