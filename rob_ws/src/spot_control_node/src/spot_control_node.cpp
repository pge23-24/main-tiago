#include "spot_control_node.hpp"

int serial;

void cmd_vel_callback(const geometry_msgs::Twist& msg )
{
  if (msg.linear.x != 0.0 || msg.linear.y != 0.0 || msg.angular.z != 0.0)
  {
    const char *msg = "1";
    write(serial, msg, 1);
  }
  else
  {
    const char *msg = "0";
    write(serial, msg, 1);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spot_control_node");
  
  ros::NodeHandle n;

  serial = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
  if (serial == -1)
  {
    // Handle error
    perror("open_port: Unable to open /dev/ttyUSB0 - ");
  }
  else
  {
    struct termios options;
    tcgetattr(serial, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    tcsetattr(serial, TCSANOW, &options);
  }

  ros::Subscriber sub = n.subscribe("/mobile_base_controller/cmd_vel", 1000, cmd_vel_callback);

  ros::spin();

  return 0;
}