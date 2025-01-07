#include <memory>
#include <unistd.h>
#include <stdlib.h>

#include <SDL2/SDL.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include<arp/InteractiveMarkerServer.hpp>
#include <arp/Autopilot.hpp>
#include <arp/cameras/PinholeCamera.hpp>
#include <arp/StatePublisher.hpp>
#include <arp/VisualInertialTracker.hpp>

#define FONT cv::FONT_HERSHEY_COMPLEX_SMALL
#define FONT_COLOR cv::Scalar(0, 255, 0)
#define FONT_OFFSET 20
#define FONT_OFFSET_PER_LETTER 11

class Subscriber
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if(!vit_)
      return;
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll + msg->header.stamp.nsec / 1000;
    std::lock_guard<std::mutex> l(imageMutex_);
    lastImage_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    vit_->addImage(timeMicroseconds, lastImage_);
  }

  bool getLastImage(cv::Mat& image)
  {
    std::lock_guard<std::mutex> l(imageMutex_);
    if(lastImage_.empty())
      return false;
    image = lastImage_.clone();
    lastImage_ = cv::Mat();  // clear, only get same image once.
    return true;
  }

  void setVisualInertialTracker(arp::VisualInertialTracker* vit){
    vit_ = vit;
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    if(!vit_)
      return;
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll + msg->header.stamp.nsec / 1000;
    Eigen::Vector3d acc_S, omega_S;
    acc_S << msg->linear_acceleration.x,
             msg->linear_acceleration.y,
             msg->linear_acceleration.z;
    omega_S << msg->angular_velocity.x,
               msg->angular_velocity.y,
               msg->angular_velocity.z;
    vit_->addImuMeasurement(timeMicroseconds, omega_S, acc_S);
  }

 private:
  cv::Mat lastImage_;
  std::mutex imageMutex_;
  arp::VisualInertialTracker* vit_ = nullptr;
};

/// \brief Read occupancy grid from file.
/// \param filename File to read.
bool readOccupancyMap(const std::string& filename, cv::Mat& occupancyMap)
{
  // open the file:
  std::ifstream mapFile(filename, std::ios::in | std::ios::binary);
  if(!mapFile.is_open()) {
    ROS_FATAL_STREAM("could not open map file " << filename);
    return false;
  }
  // first read the map size along all the dimensions:
  int mapSizes[3];
  if(!mapFile.read((char *)mapSizes, 3* sizeof(int))) {
    ROS_FATAL_STREAM("could not read map file " << filename);
    return false;
  }
  
  // create the returned cv::Mat:
  occupancyMap = cv::Mat(3, mapSizes, CV_8SC1);

  // now read the map data: don’t forget to delete[] in the end!
  int numCells = mapSizes[0] * mapSizes[1] * mapSizes[2];
  if(!mapFile.read((char *)occupancyMap.data, numCells)) {
    ROS_FATAL_STREAM("could not read map file " << filename);
    return false;
  }
  mapFile.close();
  return true;
}

/// \brief Annotate the image with the state of the autopilot, for debugging.
/// @param[in] image Image to annotate, will be overwritten.
/// @param[in] autopilot Autopilot for retrieving state information.
void annotateImage(cv::Mat& image, arp::Autopilot& autopilot)
{
  const auto status = autopilot.getDroneStatus();
  if(status == arp::Autopilot::DroneStatus::Unknown) {
    ROS_DEBUG("Drone status unknown");
    return;
  }
  // TODO: display instructions of use onto the graphics window
  cv::putText(image, "Press 'T' to take off", cv::Point(10, 20), FONT, 0.5, FONT_COLOR);
  cv::putText(image, "Press 'L' to land", cv::Point(10, 40), FONT, 0.5, FONT_COLOR);
  cv::putText(image, "Press 'C' to calibrate", cv::Point(10, 60), FONT, 0.5, FONT_COLOR);
  cv::putText(image, "Press 'U' to toggle undistortion", cv::Point(10, 80), FONT, 0.5, FONT_COLOR);
  cv::putText(image, "Press 'ESC' to stop", cv::Point(10, 100), FONT, 0.5, FONT_COLOR);

  // TODO: display the battery state of charge in the graphics window
  cv::putText(image, "Battery: " + std::to_string(autopilot.getBatteryStatus()) , cv::Point(10, 120), FONT, 0.5, FONT_COLOR);

  // TODO: display the current drone status in the graphics window
  cv::putText(image, "Status: " + std::to_string(status), cv::Point(10, 140), FONT, 0.5, FONT_COLOR);

  // TODO: print drone commands in terminal window
  const Uint8 *state = SDL_GetKeyboardState(nullptr);
  if(state[SDL_SCANCODE_ESCAPE])
  {
    cv::putText(image, "ESTOP PRESSED, SHUTTING OFF ALL MOTORS", cv::Point(10, 160), FONT, 0.5, FONT_COLOR);
  }
  if(state[SDL_SCANCODE_T])
  {
    cv::putText(image, "T PRESSED, Taking off...                          ", cv::Point(10, 180), FONT, 0.5, FONT_COLOR);
  }

  if(state[SDL_SCANCODE_L])
  {
    cv::putText(image, "L PRESSED, Going to land...                       ", cv::Point(10, 200), FONT, 0.5, FONT_COLOR);
    ROS_INFO_STREAM("Going to land...                       ");
  }
  if(state[SDL_SCANCODE_C])
  {
    cv::putText(image, "C PRESSED, Requesting flattrim calibration...     ", cv::Point(10, 200), FONT, 0.5, FONT_COLOR);
  }
  if(state[SDL_SCANCODE_U])
  {
    cv::putText(image, "U PRESSED, Toggling image undistortion...      ", cv::Point(10, 200), FONT, 0.5, FONT_COLOR);
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rmil_drone_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  double fu, fv, cu, cv, k1, k2, p1, p2, focalLengthMap;
  if(!ros::param::get("~fu", fu)) ROS_FATAL("error loading parameter fu");
  if(!ros::param::get("~fv", fv)) ROS_FATAL("error loading parameter fv");
  if(!ros::param::get("~cu", cu)) ROS_FATAL("error loading parameter cu");
  if(!ros::param::get("~cv", cv)) ROS_FATAL("error loading parameter cv");
  if(!ros::param::get("~k1", k1)) ROS_FATAL("error loading parameter k1");
  if(!ros::param::get("~k2", k2)) ROS_FATAL("error loading parameter k2");
  if(!ros::param::get("~p1", p1)) ROS_FATAL("error loading parameter p1");
  if(!ros::param::get("~p2", p2)) ROS_FATAL("error loading parameter p2");
  if(!ros::param::get("~focalLengthMap", focalLengthMap)) ROS_FATAL("error loading parameter focalLengthMap");

  std::string occupancy_map;
  if(!ros::param::get("~occupancymap", occupancy_map)) ROS_FATAL("error loading parameter occupancy_map");
  std::string path = ros::package::getPath("rmil_drone_practicals");
  std::string occupancy_map_path = path + "/maps/" + occupancy_map;
  std::cout<<occupancy_map_path<<std::endl;

  //std::string path = ros::package::getPath("rmil_drone_practicals");
  std::string mapFile;
  if(!ros::param::get("~map", mapFile))
    ROS_FATAL("error loading parameter for map");
  std::string mapPath = path + "/maps/" + mapFile;
  

  Eigen::Matrix4d T_SC_mat;
  std::vector<double> T_SC_array;
  if(!ros::param::get("~T_SC", T_SC_array))
    ROS_FATAL("error loading parameter T_SC");
  T_SC_mat <<
    T_SC_array[0], T_SC_array[1], T_SC_array[2], T_SC_array[3],
    T_SC_array[4], T_SC_array[5], T_SC_array[6], T_SC_array[7],
    T_SC_array[8], T_SC_array[9], T_SC_array[10], T_SC_array[11],
    T_SC_array[12], T_SC_array[13], T_SC_array[14], T_SC_array[15];
  arp::kinematics::Transformation T_SC(T_SC_mat);

  double briskUniformityRadius, briskAbsoluteThreshold;
  int briskMatchingThreshold, briskOctaves, briskMaxNumKeypoints;
  float reprojectionDistThreshold;
  if(!ros::param::get("~briskMatchingThreshold", briskMatchingThreshold)) ROS_FATAL("error loading parameter briskMatchingThreshold");
  if(!ros::param::get("~briskUniformityRadius", briskUniformityRadius)) ROS_FATAL("error loading parameter briskUniformityRadius");
  if(!ros::param::get("~briskOctaves", briskOctaves)) ROS_FATAL("error loading parameter briskOctaves");
  if(!ros::param::get("~briskAbsoluteThreshold", briskAbsoluteThreshold)) ROS_FATAL("error loading parameter briskAbsoluteThreshold");
  if(!ros::param::get("~briskMaxNumKeypoints", briskMaxNumKeypoints)) ROS_FATAL("error loading parameter briskMaxNumKeypoints");
  if(!ros::param::get("~reprojectionDistanceThreshold", reprojectionDistThreshold)) ROS_FATAL("error loading parameter reprojectionDistanceThreshold");

  // Global settings.
  bool isImageUndistorted = false;




  // set up autopilot and visual-inertial tracking.
  arp::Autopilot autopilot(nh);
  arp::InteractiveMarkerServer markerServer(autopilot);
  //initialize the marker to [0 0 5]
  
  
  arp::Frontend frontend(640, 360, fu, fv, cu, cv, k1, k2, p1, p2, focalLengthMap,
                         briskMatchingThreshold, briskUniformityRadius, briskOctaves,
                         briskAbsoluteThreshold, briskMaxNumKeypoints, reprojectionDistThreshold);
  if(!frontend.loadMap(mapPath))
    ROS_FATAL_STREAM("could not load map from " << mapPath << " !");
  //if(!frontend.loadMap(occupancy_map_path))
    //ROS_FATAL_STREAM("could not load map from " << occupancy_map_path << " !");

    cv::Mat map;
    readOccupancyMap(occupancy_map_path, map);
    std::cout<<"Occupancy Map Read"<<std::endl;
    autopilot.setoccupancymap(map);
    //     if (map.empty()) {
    //     std::cerr << "Error: Map is empty!" << std::endl;
    // } else {
    //     // 打印地图的尺寸和类型
    //     std::cout << "Map dimensions: ";
    //     for (int i = 0; i < map.dims; ++i) {
    //         std::cout << map.size[i] << " ";
    //     }
    //     std::cout << std::endl;

    //     std::cout << "Map type: " << map.type() << std::endl;

    //     // 遍历三维地图
    //     for (int i = 0; i < map.size[0]; ++i) {
    //         for (int j = 0; j < map.size[1]; ++j) {
    //             for (int k = 0; k < map.size[2]; ++k) {
    //                 int8_t value = map.at<int8_t>(i, j, k); // 访问栅格值
    //                 if (value != 0) { // 只打印非 0 的值
    //                 std::cout << "Value at (" << i << ", " << j << ", " << k << "): "
    //                           << static_cast<int>(value) << std::endl;
    //             }
    //         }
    //     }
    // }
    // }
  std::string vocPath = path + "/maps/small_voc.yml.gz";
  if(!frontend.loadDBoW2Voc(vocPath))
    ROS_FATAL_STREAM("could not load DBoW vocabulary from " << vocPath << " !");
  if(!frontend.fillDBoW2Database())
    ROS_FATAL_STREAM("could not fill DBoW2 with map landmarks!");

  arp::ViEkf viEkf;
  viEkf.setCameraExtrinsics(T_SC);
  viEkf.setCameraIntrinsics(frontend.camera());

  arp::VisualInertialTracker visualInertialTracker;
  visualInertialTracker.setFrontend(frontend);
  visualInertialTracker.setEstimator(viEkf);

  // setup inputs and outputs.
  Subscriber subscriber;
  subscriber.setVisualInertialTracker(&visualInertialTracker);
  image_transport::Subscriber subImage = it.subscribe("ardrone/front/image_raw", 2, &Subscriber::imageCallback, &subscriber);
  ros::Subscriber subImu = nh.subscribe("ardrone/imu", 50, &Subscriber::imuCallback, &subscriber);
  arp::StatePublisher pubState(nh);
  visualInertialTracker.setVisualisationCallback(std::bind( &arp::StatePublisher::publish, &pubState, std::placeholders::_1, std::placeholders::_2));
  
  SDL_Event event;
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window * window = SDL_CreateWindow("Hello AR Drone", SDL_WINDOWPOS_UNDEFINED,
                                         SDL_WINDOWPOS_UNDEFINED, 640, 480, 0);
  SDL_Renderer * renderer = SDL_CreateRenderer(window, -1, 0);
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_RenderClear(renderer);
  SDL_RenderPresent(renderer);
  SDL_Texture * texture;

  // enter main event loop
  ROS_INFO_STREAM("===== Hello AR Drone ====");
  cv::Mat image;
  std::vector <Eigen::Vector3d> trajectory;
  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration dur(0.04);
    dur.sleep();
    SDL_PollEvent(&event);
    if(event.type == SDL_QUIT) {
      break;
    }

    // render image, if there is a new one available
    if(visualInertialTracker.getLastVisualisationImage(image))
    {
      if(isImageUndistorted && !frontend.camera().undistortImage(image, image)) {
        ROS_WARN("Image undistortion failed");
      }
      annotateImage(image, autopilot);

      // Resize the renderer to the image size. The rendered image will be
      // scaled to fit the window.
      SDL_RenderSetLogicalSize(renderer, image.cols, image.rows);
      // https://stackoverflow.com/questions/22702630/converting-cvmat-to-sdl-texture
      // I'm using SDL_TEXTUREACCESS_STREAMING because it's for a video player, you should
      // pick whatever suits you most: https://wiki.libsdl.org/SDL_TextureAccess
      // remember to pick the right SDL_PIXELFORMAT_* !
      texture = SDL_CreateTexture(
          renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, image.cols, image.rows);
      SDL_UpdateTexture(texture, nullptr, (void*)image.data, image.step1());
      SDL_RenderClear(renderer);
      SDL_RenderCopy(renderer, texture, nullptr, nullptr);
      SDL_RenderPresent(renderer);
      // cleanup (only after you're done displaying. you can repeatedly call UpdateTexture without destroying it)
      SDL_DestroyTexture(texture);
    }

    //Multiple Key Capture Begins
    const Uint8 *state = SDL_GetKeyboardState(nullptr);

    // check states!
    auto droneStatus = autopilot.getDroneStatus();
    // command
    if(state[SDL_SCANCODE_ESCAPE])
    {
      bool success = autopilot.estopReset();
      ROS_INFO_STREAM("ESTOP PRESSED, SHUTTING OFF ALL MOTORS status=" << droneStatus << (success ? " [ OK ]" : " [FAIL]"));
    }
    if(state[SDL_SCANCODE_T])
    {
      bool success = autopilot.takeoff();
      ROS_INFO_STREAM("Taking off...                          status=" << droneStatus << (success ? " [ OK ]" : " [FAIL]"));
    }

    if(state[SDL_SCANCODE_L])
    {
      bool success = autopilot.land();
      ROS_INFO_STREAM("Going to land...                       status=" << droneStatus << (success ? " [ OK ]" : " [FAIL]"));
    }
    if(state[SDL_SCANCODE_C])
    {
      bool success = autopilot.flattrimCalibrate();
      ROS_INFO_STREAM("Requesting flattrim calibration...     status=" << droneStatus << (success ? " [ OK ]" : " [FAIL]"));
    }
    if(state[SDL_SCANCODE_U])
    {
      ROS_INFO_STREAM("Toggling image undistortion...");
      isImageUndistorted = !isImageUndistorted;
    }
    if(state[SDL_SCANCODE_RCTRL]) {
      double x, y, z, yaw;

      autopilot.getPoseReference(x, y, z, yaw);
      markerServer.activate(x, y, z, yaw);
      std::cout<<x<<" "<<y<<" "<<z<<" "<<yaw<<std::endl;

      autopilot.setAutomatic();
      ROS_INFO_STREAM("Setting automatic...     " );
      
      visualInertialTracker.setControllerCallback(
      std::bind(&arp::Autopilot::controllerCallback, &autopilot,
      std::placeholders::_1, std::placeholders::_2));
    }
    if(state[SDL_SCANCODE_SPACE]) {
      autopilot.setManual();
      ROS_INFO_STREAM("Setting manual mode...     " );
    }

    if(!autopilot.isAutomatic() && autopilot.isFlying())
    {
      // TODO: read move commands and execute them!
      double vx = 0.0, vy = 0.0, vz = 0.0, wz = 0.0;
      if(state[SDL_SCANCODE_UP]) {
        vx = 0.5;
        std::cout<<"UP"<<std::endl;
      }
      if(state[SDL_SCANCODE_DOWN]) {
        vx = -0.5;
      }
      if(state[SDL_SCANCODE_LEFT]) {
        vy = 0.5;
      }
      if(state[SDL_SCANCODE_RIGHT]) {
        vy = -0.5;
      }
      if(state[SDL_SCANCODE_W]) {
        vz = 0.5;
      }
      if(state[SDL_SCANCODE_S]) {
        vz = -0.5;
      }
      if(state[SDL_SCANCODE_A]) {
        wz = 0.5;
      }
      if(state[SDL_SCANCODE_D]) {
        wz = -0.5;
      }
      bool success = autopilot.manualMove(vx, vy, vz, wz);
      ROS_INFO_STREAM("moving the drone...     status=" << droneStatus << (success ? " [ OK ]" : " [FAIL]"));
    }
  } 

  // make sure to land the drone...
  bool success = autopilot.land();

  // cleanup
  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

