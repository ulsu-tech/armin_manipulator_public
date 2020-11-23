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
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <ros/ros.h>

#include "joint_states_keeper.h"

int read_event(int fd, struct js_event *event)
{
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    if (bytes == -1 && (errno == EAGAIN || errno == EWOULDBLOCK))
        return 1;

    /* Error, could not read full event. */
    return -1;
}

size_t get_axis_state(struct js_event *event, struct axis_state axes[3])
{
    size_t axis = event->number / 2;

    if (axis < 3)
    {
        if (event->number % 2 == 0)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }

    return axis;
}

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char** argv){
    struct js_event event;
    struct axis_state axes[3] = {0};
    size_t axis;

    ros::init(argc, argv, "armin_joystick_node");
   
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    ros::Duration period(1.0/200); // 200Hz update rate

signal(SIGINT, mySigintHandler);

    int fd = open("/dev/input/js0", O_RDONLY | O_NDELAY);
    if (fd < 0) {
        ROS_ERROR("failed to open joystick device /input/js0. Exiting");
        exit(0);
    }

    JointStatesKeeper sc{nh};
    ros::Subscriber sub = nh.subscribe("joint_states", 1,
        &JointStatesKeeper::jointStatesCallback, &sc);

    ros::Publisher chatter_pub = nh.advertise<trajectory_msgs::JointTrajectory>
            ("/armin/controller/position/command", 1);
    ros::Publisher gripper_pub = nh.advertise<std_msgs::String>("/wrist", 1);


    ros::ServiceServer executeCommandService = nh.advertiseService("change_setting",
        &JointStatesKeeper::setPlanningOptionCallback, &sc);
    ros::ServiceServer getSettingService = nh.advertiseService("read_setting",
        &JointStatesKeeper::givePlanningOptionCallback, &sc);


    while(ros::ok()){
        //read joystick
        if (read_event(fd, &event) == 0) {
             switch (event.type)
          {
            case JS_EVENT_BUTTON:
                printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
                sc.updateButtons(event.number, event.value);
                break;
            case JS_EVENT_AXIS:
                axis = get_axis_state(&event, axes);
                if (axis < 3)
                    printf("Axis %zu at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);
                sc.updateAxesState(axes);
                break;
            default:
                /* Ignore init events. */
                break;
          }
          fflush(stdout);
        }
        if ( sc.isInitialized())
            sc.publishJointsCommand(chatter_pub);
        // if need an update - publish it
        sc.publishGripCmd(gripper_pub);
        period.sleep();
    }

    spinner.stop();
    close(fd);
return 0;
}

