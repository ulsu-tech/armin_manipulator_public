#include <stdlib.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <signal.h>
#include <errno.h>

#include <sys/time.h>

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include "std_msgs/String.h"

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include "joint_states_keeper.h"

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char** argv){

    ros::init(argc, argv, "trajectory_research_node");
   
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    ros::Duration period(1.0/25); // 200Hz update rate

signal(SIGINT, mySigintHandler);

    JointStatesKeeper sc{nh};
    ros::Subscriber sub = nh.subscribe("joint_states", 1,
        &JointStatesKeeper::jointStatesCallback, &sc);

    ros::Publisher chatter_pub = nh.advertise<trajectory_msgs::JointTrajectory>
            ("/armin/controller/position/command", 1);

    sc.updateButtons(4, 1);
    sc.updateButtons(4, 0);

    while(ros::ok()){
        struct data fetchedData;
        sc.updateSensorsAndRest(fetchedData);
        if ( sc.isInitialized())
            //sc.publishJointsCommand(chatter_pub);
            sc.publishSphereJointsCommand(chatter_pub);
        // if need an update - publish it
        period.sleep();
    }

    spinner.stop();
return 0;
}

