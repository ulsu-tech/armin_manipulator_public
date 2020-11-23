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
#include "std_msgs/Int32MultiArray.h"
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
#include "custom_types.h"

#define PUBLISH_PERIOD_TO_SPHERE_MS  200

#define BUFF_SIZE 2048
char temp_buffer[BUFF_SIZE];
int tBuffFillSize = 0;

struct termios prev_tty, tty_sets;


static const uint16_t crc16tab[256]= {
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
    0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
    0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
    0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
    0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
    0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
    0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
    0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
    0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
    0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
    0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
    0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
    0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
    0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
    0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
    0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
    0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
    0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
    0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
    0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
    0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
    0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
    0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
    0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
    0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
    0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
    0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
    0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
    0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
    0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
    0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

uint16_t crc16(const char *buf, int len) {
    int counter;
    uint16_t crc = 0;
    for (counter = 0; counter < len; counter++)
            crc = (crc<<8) ^ crc16tab[((crc>>8) ^ *buf++)&0x00FF];
    return crc;
}

uint16_t crc16_old(const char *buf, int len) {
    const uint8_t *pdata = reinterpret_cast<const uint8_t*>(buf);
    uint16_t crc = 0;

    for( ; len > 0; len--, pdata++) {
        crc += (*pdata) * 44111;
       //printf( "crc: crc=0x%04x\n", crc);
    }

    return crc ^ (crc >> 8);

}

void projectUnclinedToDecart(double &t1, double &t2, double &t3)
{
    const double x1min = -8.0;
    const double x1max = -8.0;
    const double y1min = -411.19566; 
    const double y1max = -381.38839;
    const double z1min = 366.13891;
    const double z1max = 706.83750;

    // Left guide limits
    const double x2min = -356.14606;
    const double x2max = -136.04091;
    const double y2min = 50.08337;
    const double y2max = -198.92575;
    const double z2min = 796.28856;
    const double z2max = 876.98666;

    // Right guide limits
    const double x3min = 364.14606;
    const double x3max = 144.04091;
    const double y3min = 39.94943;
    const double y3max = -209.05969;
    const double z3min =  786.83851;
    const double z3max = 867.53661;

    const double x1_ = x1max - x1min;
    const double y1_ = y1max - y1min;
    const double z1_ = z1max - z1min;

    const double x2_ = x2max - x2min;
    const double y2_ = y2max - y2min;
    const double z2_ = z2max - z2min;

    const double x3_ = x3max - x3min;
    const double y3_ = y3max - y3min;
    const double z3_ = z3max - z3min;
    double l1 = sqrt( x1_ * x1_ + y1_ * y1_ + z1_ * z1_ );
    double l2 = sqrt( x2_ * x2_ + y2_ * y2_ + z2_ * z2_ );
    double l3 = sqrt( x3_ * x3_ + y3_ * y3_ + z3_ * z3_ );


    double x_real = t1 * x1_ / l1 + t2 * x2_ / l2 + t3 * x3_ / l3;
    double y_real = t1 * y1_ / l1 + t2 * y2_ / l2 + t3 * y3_ / l3;
    double z_real = t1 * z1_ / l1 + t2 * z2_ / l2 + t3 * z3_ / l3;

    t1 = x_real;
    t2 = y_real;
    t3 = z_real;
}

int checkPossibleMessage(const char *data, const int maxSize)
{
    //for now only data_pos messages are held
    if ( data[0] == 0x50 && maxSize >= sizeof(PositionalData))
    {
        auto mSize =  sizeof(PositionalData) - sizeof(uint16_t);
        auto crc_att = crc16(data, mSize);
        if (crc_att != *(reinterpret_cast<const uint16_t*>(data +mSize)))
        {
            ROS_INFO_STREAM(" on CRC check received "<<crc_att<<"  VS  "<<
                *(reinterpret_cast<const uint16_t*>(data +mSize)));
            return -1;
        }
        return sizeof(PositionalData); //sizeof( DATA_POS_struct)
    }
    return -1;
}

void fillXYZfromDATA_POS_message(const char *data, int &x, int &y, int&z)
{
    struct data const &received(*(reinterpret_cast<struct data const *>(data)));
    x = received.tenzor[0];
    y = received.tenzor[1];
    z = received.tenzor[2];
	ROS_INFO_STREAM("after message received x="<<x<<"  y="<<y<<"   z="<<z);
}

void fillENCODERSfromDATA_POS_message(const char * data,
    int &enc1, int &enc2, int &enc3)
{
    struct data const &received(*(reinterpret_cast<struct data const *>(data)));
    enc1 = received.enc1;
    enc2 = received.enc2;
    enc3 = received.enc3;
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

#define COEFFS_GROUPS_COUNT 1
#define ENCODERS_COUNT 3

double getOffsetForTensor(int enc1, int enc2, int enc3, int section)
{
    double coeefs[COEFFS_GROUPS_COUNT][ENCODERS_COUNT][10] = {
 {
    // group from sheet _169
  { -0.000913706649138, 0.000434903502618, -0.000556441802738,
        2.31967119520419E-08, -5.07401674502037E-09, -4.89143881032874E-09,
            -1.76749837195104E-13, -4.02765728010614E-14, -4.0521925849584E-14,
                2094.09144625223
  },
  { -0.000745675911236, -0.000715332750091, -0.002733108263843,
        2.23905766225519E-08, 4.60309161866856E-08, 1.73359974869515E-08,
            -1.03788460536689E-13, -1.02401949809474E-14,  -1.71386773039631E-13,
                1921.07536631099
  },
  { -0.000186923942308, -0.0008688594942, 0.000568520513093,
        1.11320363686498E-08, 2.80726432823766E-09, 2.78603840984655E-08,
            -9.71231582636021E-14, -9.58559147307749E-14, 7.52115267113347E-14,
            1818.88019030295
  }
 }
    };

    if (section <0 || section >= ENCODERS_COUNT)
        return -1.;

    int usedGroup = 0;

    return coeefs[usedGroup][section][9] +
        coeefs[usedGroup][section][0] * enc1 +
        coeefs[usedGroup][section][1] * enc2 +
        coeefs[usedGroup][section][2] * enc3 +
        coeefs[usedGroup][section][3] * enc1 * enc1 +
        coeefs[usedGroup][section][4] * enc2 * enc2 +
        coeefs[usedGroup][section][5] * enc3 * enc3;
        coeefs[usedGroup][section][6] * enc1 * enc1 * enc1 +
        coeefs[usedGroup][section][7] * enc2 * enc2 * enc2 +
        coeefs[usedGroup][section][8] * enc3 * enc3 * enc3;
}

double tensorZeroThreshold = 150;

void cutThreshold(double &v) {
        if (abs(v) > tensorZeroThreshold) {
            v -= (v>0?tensorZeroThreshold:-tensorZeroThreshold);
        } else {
            v = 0;
        }
    };

int read_event(int fd, struct axis_state axes[3], struct data & d_ptr)
{
    ssize_t bytes;
    bool rv = false;

    // forcing button not to have user_presence flag in case a cll will fail
   d_ptr.button &= ~(BUTTONS_USER_PRESENCE);

    bytes = read(fd, temp_buffer + tBuffFillSize, BUFF_SIZE - tBuffFillSize);

    if (bytes == -1 && (errno == EAGAIN || errno == EWOULDBLOCK))
        return 1;

    // now let's check if there's valuable data
    if (bytes > 0)
        tBuffFillSize += bytes;
    int x,y,z;
    int enc1, enc2, enc3;
    if (tBuffFillSize > 0)
    {
        for(int i=0; i < tBuffFillSize; ++i)
        {
            auto validSize = checkPossibleMessage(temp_buffer + i, tBuffFillSize - i);
            if (validSize > 0) {
                memcpy(&d_ptr, temp_buffer + i, sizeof (d_ptr));
                fillXYZfromDATA_POS_message(temp_buffer + i, x,y,z);
                fillENCODERSfromDATA_POS_message(temp_buffer + i,
                    enc1, enc2, enc3);
                SHL(i + validSize);
                rv = true;
                break;
            }
        }
    }
    if (rv) {
        double t1 = getOffsetForTensor(enc1, enc2, enc3, 0);
        double t2 = getOffsetForTensor(enc1, enc2, enc3, 1);
        double t3 = getOffsetForTensor(enc1, enc2, enc3, 2);
        t1 = x - t1;
        t2 = y - t2;
        t3 =  z - t3;
        cutThreshold(t1);
        cutThreshold(t2);
        cutThreshold(t3);
        // TODO convert values from oblique coordinate system to "normal"
        projectUnclinedToDecart(t1, t2, t3);
        // for drawing
        x = t1;
        y = t2;
        z = t3;

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

int timespec_diff_ms(const struct timespec &a, const struct timespec &b)
{
    int v = a.tv_sec - b.tv_sec;
    v *= 1000;
    v += (a.tv_nsec - b.tv_nsec)/1000000;
    return v;
}

int main(int argc, char** argv){
    struct js_event event;
    struct axis_state axes[3] = {0};
    size_t axis;

    ros::init(argc, argv, "armin_sphere_tensored_node");
   
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    ros::Duration period(1.0/200); // 200Hz update rate

signal(SIGINT, mySigintHandler);

    int fd = open("/dev/ttyUSB0", O_RDWR | O_NDELAY);
    if (fd < 0) {
        ROS_ERROR("failed to open sphere device /dev/ttyUSB0. Exiting");
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
    ros::Publisher latest_tensors_pub = nh.advertise<std_msgs::Int32MultiArray>("/tensor_tension", 1);
    ros::ServiceServer executeCommandService = nh.advertiseService("change_setting",
        &JointStatesKeeper::setPlanningOptionCallback, &sc);
    ros::ServiceServer getSettingService = nh.advertiseService("read_setting",
        &JointStatesKeeper::givePlanningOptionCallback, &sc);

    std_msgs::Int32MultiArray tensor_values;
    tensor_values.layout.dim.resize(1);
    tensor_values.layout.dim[0].label = "tensors";
    tensor_values.layout.dim[0].size = 3;
    tensor_values.layout.dim[0].stride = 3;
    tensor_values.layout.data_offset = 0;
    tensor_values.data.resize(3);


    struct data lastFetchedData;

    struct timespec thisMoment, lastSendMoment;
    clock_gettime(CLOCK_MONOTONIC_RAW, &thisMoment);
    lastSendMoment = thisMoment;
    bool prevUserPresent = false;
    while(ros::ok()){
        //read joystick
	    bool readingSuccessful = (read_event(fd, axes, lastFetchedData) == 0);
        if (readingSuccessful) {
                // rotating ax[1].y and ax[0].x
                struct axis_state tmp = axes[0];
                axes[0].x = axes[1].y;
                axes[1].y = -tmp.x;
		axes[0].x *=3;
		axes[0].y *=3;
		axes[1].y *=3;
                sc.updateAxesState(axes);
                sc.updateSensorsAndRest(lastFetchedData);
            // emulating only orientation treat
            sc.updateButtons(6, IS_BUTTON_CLOSE(lastFetchedData));
            // emulating allow orientation change
            sc.setAllowOrientationChange(IS_BUTTON_OPEN(lastFetchedData));
        }
        if ( sc.isInitialized() && IS_USER_PRESENT(lastFetchedData))
        {
            sc.publishSphereJointsCommand(chatter_pub);
        }
        if ( readingSuccessful && (! IS_USER_PRESENT(lastFetchedData)) && prevUserPresent)
        {
            sc.publishStop(chatter_pub, "main() function");
        }
	if (readingSuccessful)
	{
           prevUserPresent =  IS_USER_PRESENT(lastFetchedData);
	}
            tensor_values.data[0] = static_cast<int>(-axes[1].y * 2048. /32000.);
            tensor_values.data[1] = static_cast<int>( axes[0].x * 2048. /32000.);
            tensor_values.data[2] = static_cast<int>(-axes[0].y * 2048. /32000.);

            latest_tensors_pub.publish(tensor_values);

        if (sc.isInitialized())
        {
            clock_gettime(CLOCK_MONOTONIC_RAW, &thisMoment);
            if (timespec_diff_ms(thisMoment, lastSendMoment) > PUBLISH_PERIOD_TO_SPHERE_MS)
            {
                ToolPositionInform message;
                message.tag = 0x80;
                sc.getLatestPosition_1over10_mm(message.x, message.y, message.z);
		auto mx = message.x;
		message.x = -message.y;
		message.y = mx;
                message.crc = crc16( reinterpret_cast<char *>(&message), sizeof(message) - sizeof(message.crc));
		std::cout<<"Sending values x="<<message.x<<"   y="<<message.y<<"   z="<<message.z<<std::endl;
                int v;
		v = write(fd, &message, sizeof(message));
                if (v!= sizeof(message))
                {
                    std::cout<<"send only "<<v<<" bytes "
                        <<"out of "<<sizeof(message)<<"  required"<<std::endl;
                }
		lastSendMoment = thisMoment;
            }
        }
        //
        period.sleep();
    }

    spinner.stop();
    close(fd);
return 0;
}

