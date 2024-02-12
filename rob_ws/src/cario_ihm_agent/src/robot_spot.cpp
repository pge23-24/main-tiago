// Copyright 2024 cario360

#include "robot_spot.hpp"

std::unique_ptr<RelayControlNode> relay_controller;

RelayControlNode::RelayControlNode(ros::NodeHandle nh, const char* port, int baudrate) {
  // Initialize serial port
  this->serial = open(port, O_RDWR | O_NOCTTY);

  if (this->serial == -1) {
    std::string error_message =
        "Error opening serial port '" + std::string(port) + "'";
    ROS_ERROR(error_message.c_str());
    this->init_success = false;
    return;
  }

  // Set serial port options
  struct termios options;
  tcgetattr(serial, &options);
  cfsetispeed(&options, baudrate);
  cfsetospeed(&options, baudrate);
  tcsetattr(serial, TCSANOW, &options);

  std::string success_message =
      "Serial port '" + std::string(port) + "' opened successfully";
  ROS_INFO(success_message.c_str());

  // Initialize ROS elements
  this->sub_cmd_vel = nh.subscribe(CMD_VEL_TOPIC, 1000,
                                  &RelayControlNode::cmd_vel_callback, this);
  this->sub_joystick = nh.subscribe(JOY_TOPIC, 1000,
                                  &RelayControlNode::is_joy_actif, this);
  this->sub_recovery = nh.subscribe(RECOVERY_TOPIC, 1000, 
                                  &RelayControlNode::is_recovery_actif, this);  
  this->sub_camera_detection = nh.subscribe(CAMERA_TOPIC, 10, &RelayControlNode::camera_detection_callback, this);
  
  this->init_success = true;

  // Initialize buffers
  this->init_buffer();

  this->human_in_fov = false;
}

void RelayControlNode::camera_detection_callback(const multi_obstacles_tracker_msgs::CameraDetectionStampedArray& msg)
{
  bool res = false;
  for(auto i = 0; i < msg.detections.size(); i++)
  {
    // ROS_INFO(
    //   "%s, %f < %f < %f", 
    //   msg.detections[i].classification.c_str(), sin((360 - HALF_FOV_DEG)*3.14/180.0), 
    //   sin(msg.detections[i].coordinates[1]*3.14/180.0),
    //   sin(HALF_FOV_DEG*3.14/180.0)
    //   );

    if(msg.detections[i].classification == "person")
    {
      if(sin((360 - HALF_FOV_DEG)*3.14/180.0) <= sin(msg.detections[i].coordinates[1]*3.14/180.0)
      && sin(HALF_FOV_DEG*3.14/180.0) >= sin(msg.detections[i].coordinates[1]*3.14/180.0))
      {
        res = true;
      }
    }
  }

  this->detection_mutex.lock();
  this->human_in_fov = res;
  this->detection_mutex.unlock();
}

void RelayControlNode::shutdown()
{
  write(this->serial, ALL_OFF, 1);
  close(this->serial);
}

void RelayControlNode::init_buffer()
{
  for(auto i = 0; i < BUFFER_SIZE; i++)
  {
    this->linear_vel_buffer[i] = 0;
    this->angular_vel_buffer[i] = 0;
  }
}

float RelayControlNode::compute_mean(float tab[])
{
  float sum = 0;

  for(auto i = 0; i < BUFFER_SIZE; i++)
  {
    sum += tab[i];
  }

  return (sum / BUFFER_SIZE);
}

void RelayControlNode::cmd_vel_callback(const geometry_msgs::Twist& msg) {
  // Mise à jour du buffer circulaire
  linear_vel_buffer[current_index] = msg.linear.x;
  angular_vel_buffer[current_index] = msg.angular.z;
  current_index = (current_index + 1) % BUFFER_SIZE;

  // Calcul moyennes glissantes vitesses
  float vit_lin = this->compute_mean(this->linear_vel_buffer);
  float vit_ang = this->compute_mean(this->angular_vel_buffer);

  // Calcul des conditions de la MAE
  bool vit_lin_null = abs(vit_lin) <= 0.01;  //vitesse linéaire
	bool vit_ang_null = abs(vit_ang) <= 0.01; //vitesse angulaire
  this->detection_mutex.lock();
  bool humain = this->human_in_fov;
  this->detection_mutex.unlock();

  // Mise à jour état MAE
  switch (spot_state) {
    case ETEINT:
      if ((!vit_lin_null || !vit_ang_null) && !humain) {
        ROS_INFO("Robot moving, spot turns ON");
        spot_state = ALLUME;
      }
      else if ((!vit_lin_null || !vit_ang_null) && humain){
				ROS_INFO("Robot moving, spot flashes");
        this->blink_state = B_ETEINT;
        this->spot_state = CLIGNOTANT;
      }      
      break;

		case CLIGNOTANT:
		if ((!vit_lin_null || !vit_ang_null) && !humain) { //bouge donc s'allume
				ROS_INFO("Robot moving, spot stays ON");
				spot_state = ALLUME;
		} 
		else if (vit_lin_null && vit_ang_null) { // time fini donc s'éteint
				ROS_INFO("Robot not moving anymore, spot turns OFF");
				spot_state = ETEINT;
		} 
		break;

    case ALLUME:
      if ((!vit_lin_null || !vit_ang_null) && humain) {
        ROS_INFO("Robot moving, spot flashes");
        this->blink_state = B_ETEINT;
        spot_state = CLIGNOTANT;
      }
			if (vit_lin_null && vit_ang_null) {
				ROS_INFO("Robot not moving anymore, spot turns OFF");
				spot_state = ETEINT ;
			}
      break;
  }

  // Actions sur états
  float elapsed_time;

  switch(spot_state)
  {
    case ETEINT:
      write(this->serial, SPOT_OFF, 1);
    break;

    case CLIGNOTANT:
      switch(blink_state) {
        case B_ETEINT:
          this->timerStart = ros::Time::now().toSec();  
          write(this->serial, SPOT_ON, 1);
          blink_state = B_WAIT1;
        break;
        
        case B_WAIT1:
          elapsed_time = ros::Time::now().toSec() - this->timerStart;
          if(elapsed_time >= SPOT_T_ON)
          {
            blink_state = B_ALLUME;
          }
        break;

        case B_ALLUME:
          this->timerStart = ros::Time::now().toSec();  
          write(this->serial, SPOT_OFF, 1);
          blink_state = B_WAIT2;
        break;

        case B_WAIT2:
          elapsed_time = ros::Time::now().toSec() - this->timerStart;
          if(elapsed_time >= SPOT_T_OFF)
          {
            blink_state = B_ETEINT;
          }
        break;
      }
    break;

    case ALLUME:
      write(this->serial, SPOT_ON, 1);
    break;
  }
}

void RelayControlNode::timer_callback(const ros::TimerEvent& event) {
  switch (mat_state) {
    case IDLE_NAV:
      if (joy_actif) {
        ROS_INFO("Joystick active, ORANGE mat turns ON");
        write(this->serial, MAT_GREEN_OFF, 1);
        write(this->serial, MAT_ORANGE_ON, 1);
        mat_state = MANUAL;
      } else if (recovery_actif) {
        ROS_INFO("Recovery active, RED mat turns ON");
        write(this->serial, MAT_GREEN_OFF, 1);
        write(this->serial, MAT_RED_ON, 1);
        mat_state = RECOVERY;
      }
      break;
    
    case MANUAL:
      if (!joy_actif && !recovery_actif) {
        ROS_INFO("Joystick inactive, ORANGE mat turns OFF");
        write(this->serial, MAT_ORANGE_OFF, 1);
        write(this->serial, MAT_GREEN_ON, 1);
        mat_state = IDLE_NAV;
      } else if (!joy_actif && recovery_actif) {
        ROS_INFO("Recovery active, RED mat turns ON");
        write(this->serial, MAT_ORANGE_OFF, 1);
        write(this->serial, MAT_RED_ON, 1);
        mat_state = RECOVERY;
      }
      break;

    case RECOVERY:
      if (!recovery_actif) {
        ROS_INFO("Recovery inactive, RED mat turns OFF");
        write(this->serial, MAT_RED_OFF, 1);
        write(this->serial, MAT_GREEN_ON, 1);
        mat_state = IDLE_NAV;
      } else if (joy_actif) {
        ROS_INFO("Joystick active, ORANGE mat turns ON");
        write(this->serial, MAT_RED_OFF, 1);
        write(this->serial, MAT_ORANGE_ON, 1);
        mat_state = MANUAL;
      }
      break;
  }
}

void RelayControlNode::is_joy_actif(const actionlib_msgs::GoalStatusArray& msg) {
  if (sig_joy == DOWN && msg.status_list.size() > 0) {
    sig_joy = UP;
    joy_actif = !joy_actif;
  } else if (sig_joy == UP && msg.status_list.size() == 0) {
    sig_joy = DOWN;
  }
}

void RelayControlNode::is_recovery_actif(const move_base_msgs::RecoveryStatus& msg) //TODO: change to recoverymessage
{
  if (msg.current_recovery_number >= 0) {
    recovery_actif = true;
    }
  else {
    recovery_actif = false;
  }
}

void sigint_handler(int sig) {
  relay_controller->shutdown();
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "relay_controller", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;
  relay_controller = std::make_unique<RelayControlNode>(nh, PORT, BAUDRATE);

  if (relay_controller->init_successful()) {
    signal(SIGINT, sigint_handler);
    ros::spin();
  }
  
  return 0;
}
