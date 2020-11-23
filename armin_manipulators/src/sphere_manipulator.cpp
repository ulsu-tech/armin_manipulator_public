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
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include "joint_states_keeper.h"
#include <termios.h>

#define BUFF_SIZE 2048
char temp_buffer[BUFF_SIZE];
int tBuffFillSize = 0;

struct termios prev_tty, tty_sets;

uint16_t crc16(const char *buf, int len) {
    const uint8_t *pdata = reinterpret_cast<const uint8_t*>(buf);
    uint16_t crc = 0;

    for( ; len > 0; len--, pdata++) {
        crc += (*pdata) * 44111;
       //printf( "crc: crc=0x%04x\n", crc);
    }

    return crc ^ (crc >> 8);

}

int checkPossibleMessage(const char *data, const int maxSize)
{
    //for now only data_pos messages are held
    if ( data[0] == 0x50 && maxSize >= 16)
    {
        auto crc_att = crc16(data, 14);
        if (crc_att != *(reinterpret_cast<const uint16_t*>(data +14)))
        {
            std::cout<<" on CRC check received "<<crc_att<<"  VS  "<<
                *(reinterpret_cast<const uint16_t*>(data +14));
            return -1;
        }
        return 16; //sizeof( DATA_POS_struct)
    }
    return -1;
}

void fillXYZfromDATA_POS_message(const char *data, int &x, int &y, int&z)
{
    const uint16_t *tensorStart = reinterpret_cast<const uint16_t*>(data + 8);
    x = tensorStart[0];
    y = tensorStart[1];
    z = tensorStart[2];
}

void shiftZero( int &x, int &y, int &z)
{
    x -= 2048;
    y -= 2048;
    z -= 2048;

    y *= -1;
    z *= -1;
    auto rec2 = 1./sqrt(2.);
    auto x_n = x * rec2 - z * rec2;
    z = + rec2 * x + rec2 * z;
    x = x_n;
}

void SHL(const int shift)
{
    if (shift == tBuffFillSize)
    {
        tBuffFillSize = 0;
        return;
    }
    memmove(temp_buffer, temp_buffer + shift, tBuffFillSize - shift);
    tBuffFillSize -= shift;
}


int read_event(int fd, struct axis_state axes[3])
{
    ssize_t bytes;
    bool rv = false;

    bytes = read(fd, temp_buffer + tBuffFillSize, BUFF_SIZE - tBuffFillSize);

    if (bytes == -1 && (errno == EAGAIN || errno == EWOULDBLOCK))
        return 1;

    // now let's check if there's valuable data
    if (bytes > 0)
        tBuffFillSize += bytes;
    int x,y,z;
    if (tBuffFillSize > 0)
    {
        for(int i=0; i < tBuffFillSize; ++i)
        {
            auto validSize = checkPossibleMessage(temp_buffer + i, tBuffFillSize - i);
            if (validSize > 0) {
                fillXYZfromDATA_POS_message(temp_buffer + i, x,y,z);
                SHL(i + validSize);
                rv = true;
            }
        }
    }
    if (rv) {
        shiftZero( x, y, z);
        axes[1].y = -x * 32000./2048;
        axes[0].x = y * 32000./2048;
        axes[0].y = -z * 32000./2048;
        return 0;
    }

    /* Error, could not read full event. */
    return -1;
}

void setUpSerial(int fd)
{
    //
    memset(&prev_tty, 0, sizeof(struct termios));
    memset(&tty_sets, 0, sizeof(struct termios));
    tcgetattr(fd, &prev_tty);
    tcgetattr(fd, &tty_sets);
    tty_sets.c_cflag &= ~PARENB;
    tty_sets.c_cflag &= ~CSTOPB;
    tty_sets.c_cflag |= CS8;
    tty_sets.c_cflag &= ~CRTSCTS;
    tty_sets.c_cflag |= CREAD | CLOCAL;
    tty_sets.c_lflag &= ~ICANON;
    tty_sets.c_lflag &= ~(ECHO | ECHOE| ECHONL | ISIG );
    tty_sets.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty_sets.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty_sets.c_oflag &= ~( OPOST | ONLCR);
    tty_sets.c_cc[VTIME] = 10;
    tty_sets.c_cc[VMIN] = 0;
    cfsetospeed(&tty_sets, B115200);
    if (tcsetattr(fd, TCSANOW, &tty_sets) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
}
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

    ros::init(argc, argv, "armin_sphere_node");
   
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    ros::Duration period(1.0/200); // 200Hz update rate

signal(SIGINT, mySigintHandler);

    int fd = open("/dev/ttyUSB0", O_RDONLY | O_NDELAY);
    if (fd < 0) {
        ROS_ERROR("failed to open sphere device /input/js0. Exiting");
        exit(0);
    } else {
        setUpSerial(fd);
    }

    JointStatesKeeper sc{nh};
    sc.updateButtons(4, 1);
    sc.updateButtons(4, 0);

    ros::Subscriber sub = nh.subscribe("joint_states", 1,
        &JointStatesKeeper::jointStatesCallback, &sc);

    ros::Publisher chatter_pub = nh.advertise<trajectory_msgs::JointTrajectory>
            ("/armin/controller/position/command", 1);
    ros::Publisher gripper_pub = nh.advertise<std_msgs::String>("/wrist", 1);

    while(ros::ok()){
        //read joystick
        if (read_event(fd, axes) == 0) {
                sc.updateAxesState(axes);
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


