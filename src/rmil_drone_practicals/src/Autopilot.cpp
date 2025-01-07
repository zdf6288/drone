/*
 * Autopilot.cpp
 *
 *  Created on: 10 Jan 2017
 *      Author: sleutene
 */

#include <geometry_msgs/PoseStamped.h>

#include <arp/Autopilot.hpp>
#include <arp/kinematics/operators.hpp>

namespace arp {

Autopilot::Autopilot(ros::NodeHandle& nh)
    : nh_(&nh)
{
  // Receiving navdata.
  subNavdata_ = nh.subscribe("ardrone/navdata", 50, &Autopilot::navdataCallback, this);
  // Publishers.
  isAutomatic_ = false; // always start in manual mode

  // commands
  pubReset_ = nh_->advertise<std_msgs::Empty>("/ardrone/reset", 1);
  pubTakeoff_ = nh_->advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  pubLand_ = nh_->advertise<std_msgs::Empty>("/ardrone/land", 1);
  pubMove_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  // flattrim service
  srvFlattrim_ = nh_->serviceClient<std_srvs::Empty>(nh_->resolveName("ardrone/flattrim"), 1);
}

void Autopilot::navdataCallback(const ardrone_autonomy::NavdataConstPtr& msg)
{
  std::lock_guard<std::mutex> l(navdataMutex_);
  lastNavdata_ = *msg;
}

// Request flattrim calibration.
bool Autopilot::flattrimCalibrate()
{
  DroneStatus status = getDroneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> flattrim calibrate
  std_srvs::Empty flattrimServiceRequest;
  srvFlattrim_.call(flattrimServiceRequest);
  return true;
}

// Takeoff.
bool Autopilot::takeoff()
{
  DroneStatus status = getDroneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> take off
  std_msgs::Empty takeoffMsg;
  pubTakeoff_.publish(takeoffMsg);
  return true;
}

// Land.
bool Autopilot::land()
{
  DroneStatus status = getDroneStatus();
  if (status != DroneStatus::Landed && status != DroneStatus::Landing
      && status != DroneStatus::Looping) {
    // ARdrone -> land
    std_msgs::Empty landMsg;
    pubLand_.publish(landMsg);
    return true;
  }
  return false;
}

// Turn off all motors and reboot.
bool Autopilot::estopReset()
{
  // ARdrone -> Emergency mode
  std_msgs::Empty resetMsg;
  pubReset_.publish(resetMsg);
  return true;
}

Autopilot::DroneStatus Autopilot::getDroneStatus()
{
  ardrone_autonomy::Navdata navdata;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
    navdata = lastNavdata_;
  }
  return DroneStatus(navdata.state);
}

float Autopilot::getBatteryStatus() const
{
  return lastNavdata_.batteryPercent;
}

bool Autopilot::isFlying() const
{
  const uint status = lastNavdata_.state;
  return status == 3 || status == 4 || status == 7;
}

bool Autopilot::isLanded() const
{
  const uint status = lastNavdata_.state;
  return status == 2;
}

bool Autopilot::manualMove(double forward, double left, double up, double rotateLeft)
{
  if(isAutomatic()) return false;
  return move(forward, left, up, rotateLeft);
}

// Move the drone.
bool Autopilot::move(double forward, double left, double up, double rotateLeft)
{ 

  //TODO Implement movement of the drone
  
  DroneStatus status = getDroneStatus();

  // Create the command
  // ros::Publisher pubMove_;
  // pubMove_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  geometry_msgs::Twist cmd;
  cmd.linear.x = forward;
  cmd.linear.y = left;
  cmd.linear.z = up;
  cmd.angular.z = rotateLeft;
  
  // Publish the command
  pubMove_.publish(cmd);
  //std::cout<<"###################### Published command"<<std::endl;
  return true;
}

// Set to automatic control mode.
void Autopilot::setManual()
{
  isAutomatic_ = false;
}

// Set to manual control mode.
void Autopilot::setAutomatic()
{
  isAutomatic_ = true;
}

// Move the drone automatically.
bool Autopilot::setPoseReference(double x, double y, double z, double yaw)
{
  std::lock_guard<std::mutex> l(refMutex_);
  
  ref_x_ = x;
  ref_y_ = y;
  ref_z_ = z;
  ref_yaw_ = yaw;

  if(!isPathClear(map_,x_,x,y,z)){
    std::cout<<"Path not clear"<<std::endl;
  } 
  
  
  //std::cout<<"###################### SetPoseReference"<<std::endl;
  std::cout<<"ref_x_ "<<ref_x_<<" ref_y_ "<<ref_y_<<" ref_z_ "<<ref_z_<<std::endl;
  return true;
}

bool Autopilot::getPoseReference(double& x, double& y, double& z, double& yaw) {
  std::lock_guard<std::mutex> l(refMutex_);
  x = ref_x_;
  y = ref_y_;
  z = ref_z_;
  yaw = ref_yaw_;

  //std::cout<<"###################### GetPoseReference"<<std::endl;
  //std::cout<<ref_x_<<" "<<ref_y_<<" "<<ref_z_<<" "<<ref_yaw_<<std::endl;

  return true;
}

bool Autopilot::isPathClear(const cv::Mat& map,const arp::kinematics::RobotState& x,double xd,double yd,double zd) {    
  //std::cout << "###################### IsPathClear" << std::endl;

    // 检查地图是否为空
    if (map.empty()) {
        std::cerr << "Error: Map is empty!" << std::endl;
        return false;
    }

    //print map size and resolution
    //std::cout << "Map size: " << map.size[0] << " x " << map.size[1] << " x " << map.size[2] << std::endl;
    double resolution = 0.1; // resolution of the map in meters
    //std::cout << "Resolution: " << resolution << std::endl;


    // current position
    double x_curr = x.t_WS[0];
    double y_curr = x.t_WS[1];
    double z_curr = x.t_WS[2];

    int steps = std::ceil(std::sqrt(
        std::pow(xd - x_curr, 2) +
        std::pow(yd - y_curr, 2) +
        std::pow(zd - z_curr, 2)) / resolution);

    for (int step = 0; step <= steps; ++step) {
    double x = x_curr + ((double)step / steps) * (xd - x_curr);
    double y = y_curr + ((double)step / steps) * (yd - y_curr);
    double z = z_curr + ((double)step / steps) * (zd - z_curr);

    std::cout <<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<xd-x_curr<<" "<<yd-y_curr<<" "<<zd-z_curr<<std::endl;

    //std::cout << "Step " << step 
              //<< ": x = " << x << ", y = " << y << ", z = " << z
              //<< ", ref_x_ = " << ref_x_ << ", ref_y_ = " << ref_y_ << ", ref_z_ = " << ref_z_
              //<< std::endl;

    int i = std::round(x/0.1+double(map.size[0]-1)/2.0);
    int j = std::round(y/0.1+double(map.size[1]-1)/2.0);
    int k = std::round(z/0.1+double(map.size[2]-1)/2.0);

    //std::cout << "i = " << i << ", j = " << j << ", k = " << k << std::endl;
    //std::cout << "Map depth: " << map.depth() << std::endl;
    //std::cout << "Map channels: " << map.channels() << std::endl;
    // std::cout << static_cast<int>(map.at<int8_t>(i, j, k)) << std::endl;
    //std::cout << "Path is clear at (i, j, k) = (" << i << ", " << j << ", " << k << ")" << std::endl;

    std::cout << "##### map value:"<< std::to_string(map.at<int8_t>(i, j, k)) << "." << std::endl;
    
    if (map.at<int8_t>(i, j, k) > -4) {
        std::cout << "map value:"<< map.at<int8_t>(i, j, k) << std::endl;
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Path not clear at (i, j, k) = (" << i << ", " << j << ", " << k << ")" << std::endl;
        //std::cout << map.at<int8_t>(i, j, k) << std::endl;
        ref_x_ = x_curr + ((double)(step-5) / steps) * (xd - x_curr);
        ref_y_ = y_curr + ((double)(step-5) / steps) * (yd - y_curr);
        ref_z_ = z_curr + ((double)(step-5) / steps) * (zd - z_curr);
        return false;
    }
}
    
    return true; // path is clear
}
void Autopilot::setoccupancymap(const cv::Mat& map){
  map_ = map;
}
/// The callback from the estimator that sends control outputs to the drone
void Autopilot::controllerCallback(uint64_t timeMicroseconds,
                                  const arp::kinematics::RobotState& x)
{ 
  x_ = x;
  // only do anything here, if automatic
  if (!isAutomatic_) {
    // keep resetting this to make sure we use the current state as reference as soon as sent to automatic mode
    const double yaw = kinematics::yawAngle(x.q_WS);
    setPoseReference(x.t_WS[0], x.t_WS[1], x.t_WS[2], yaw);
    //std::cout<<"###################### ControllerCallback(not automatic)"<<std::endl;
    return;
  }

  // TODO: only enable when in flight
  if (!isFlying()) {
    return;
  }

  //if(!isPathClear(map_,x)){
    //std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Path not clear"<<std::endl;
  //}


  // TODO: get ros parameter
  //compute the position error signal
  double x_ref, y_ref, z_ref, yaw_ref;
  getPoseReference(x_ref, y_ref, z_ref, yaw_ref);
  double e_x = x_ref - x.t_WS[0];
  double e_y = y_ref - x.t_WS[1];
  double e_z = z_ref - x.t_WS[2];
  //std::cout<<"###################### ControllerCallback(automatic)"<<std::endl;
  //std::cout<<e_x<<" "<<e_y<<" "<<e_z<<" "<<yaw_ref<<std::endl;

  //transfer to body frame with quaternion
  Eigen::Quaterniond q_WS = x.q_WS;
  Eigen::Vector3d e_body = q_WS.inverse() * Eigen::Vector3d(e_x, e_y, e_z);
  e_x = e_body[0];
  e_y = e_body[1];
  e_z = e_body[2];
  double e_yaw = yaw_ref - kinematics::yawAngle(x.q_WS);
  //the yaw error needs to be adjusted with wrap-around considerations that ensurese ψ is within the limits of [−π, π]
  if (e_yaw > M_PI) {
    e_yaw -= 2*M_PI;
  }
  if (e_yaw < -M_PI) {
    e_yaw += 2*M_PI;
  }
  //compute edot
  double e_dot_x = x.v_W[0];
  double e_dot_y = x.v_W[1];
  double e_dot_z = x.v_W[2];
  //transfer to body frame with quaternion
  Eigen::Vector3d e_dot_body = q_WS.inverse() * Eigen::Vector3d(e_dot_x, e_dot_y, e_dot_z);
  e_dot_x = e_dot_body[0];
  e_dot_y = e_dot_body[1];
  e_dot_z = e_dot_body[2];
  double e_dot_yaw = 0;
  // TODO: compute control output
  nh_->getParam("/ardrone_driver/euler_angle_max",pidX_.maxOutput_);
  nh_->getParam("/ardrone_driver/euler_angle_max",pidX_.minOutput_);

  nh_->getParam("/ardrone_driver/euler_angle_max",pidY_.maxOutput_ );
  //nh_->getParam("/ardrone_driver/euler_angle_max",pidY_.minOutput_ );

  nh_->getParam("/ardrone_driver/control_vz_max",pidZ_.maxOutput_);
  pidZ_.maxOutput_ *= 1e-3; 
  //-1e-3 * nh_->getParam("/ardrone_driver/control_vz_max",pidZ_.minOutput_);

  nh_->getParam("/ardrone_driver/control_yaw",pidYaw_.maxOutput_);
  //pidYaw_.minOutput_ = -nh_->getParam("/ardrone_driver/control_yaw");

  
  double vx = pidX_.control(timeMicroseconds, e_x, e_dot_x);
  double vy = pidY_.control(timeMicroseconds, e_y, e_dot_y);
  double vz = pidZ_.control(timeMicroseconds, e_z, e_dot_z);
  double wz = pidYaw_.control(timeMicroseconds, e_yaw, e_dot_yaw);


  // TODO: send to move
  move(vx, vy, vz, wz);
}

}  // namespace arp

