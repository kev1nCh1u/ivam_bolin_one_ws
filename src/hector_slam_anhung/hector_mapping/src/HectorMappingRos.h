//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HECTOR_MAPPING_ROS_H__
#define HECTOR_MAPPING_ROS_H__

#include "ros/ros.h"
#include "ros/package.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

//==========
#include <hector_mapping/setmap_hec.h>
#include "sensor_msgs/LaserScan.h"
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>

#include "laser_geometry/laser_geometry.h"
#include "nav_msgs/GetMap.h"

#include "slam_main/HectorSlamProcessor.h"

#include "scan/DataPointContainer.h"
#include "util/MapLockerInterface.h"

#include <std_msgs/String.h>

#include <boost/thread.hpp>



/*=====Anhung====================Socket Control ================================*/
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <fstream>

/*If you want to use SDL library, you should rewrite CMakefile */
#include <SDL/SDL_image.h>

#include "PoseInfoContainer.h"



#define P_SOCKET_BUFLEN 10

#define P_SOCKET_PORT 9930

#define P_SLAM_STATE_RUN                    0

#define P_SLAM_STATE_LOAD_MAP               1

#define P_SLAM_STATE_CREATE_MAP             2

#define P_SLAM_STATE_SAVE_MAP               3

#define P_SLAM_STATE_Re_Location            4

#define P_SLAM_STATE_Idle           	    5

#define P_MAP_STORE_PATH                    "/ros_map/"

#define P_MAP_LOAD_PATH                     "/ros_map/"

#define P_MAP_FILE_NAME                     "myMap"

//#define P_MAP_IMAGE_NAME                    "myMap_"

#define P_MAP_IMAGE_ViceName                ".pgm"
//define P_MAP_IMAGE_ViceName                ".bmp"

#define LocalparPATH_Local "/src/move_robot/parameter/local_parameter"

class HectorDrawings;
class HectorDebugInfoProvider;

class MapPublisherContainer
{
public:
  ros::Publisher mapPublisher_;
  ros::Publisher mapMetadataPublisher_;
  nav_msgs::GetMap::Response map_;
  ros::ServiceServer dynamicMapServiceServer_;
};

class HectorMappingRos
{
public:

  HectorMappingRos();

  ~HectorMappingRos();

  void scanCallback(const sensor_msgs::LaserScan& scan);

  void sysMsgCallback(const std_msgs::String& string);

  bool mapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);

  void publishMap(MapPublisherContainer& map_, const hectorslam::GridMap& gridMap, ros::Time timestamp, MapLockerInterface* mapMutex = 0);

  bool rosLaserScanToDataContainer(const sensor_msgs::LaserScan& scan, hectorslam::DataContainer& dataContainer, float scaleToMap);

  //bool rosPointCloudToDataContainer(const sensor_msgs::PointCloud& pointCloud, const tf::StampedTransform& laserTransform, hectorslam::DataContainer& dataContainer, float scaleToMap);
  bool rosPointCloudToDataContainer(const sensor_msgs::PointCloud& pointCloud_1,
									const sensor_msgs::PointCloud& pointCloud_2,
									const tf::StampedTransform& laserTransform_1,
									const tf::StampedTransform& laserTransform_2,
									hectorslam::DataContainer& dataContainer_1,
									hectorslam::DataContainer& dataContainer_2,
									float scaleToMap,
								    int device_num);

  void setServiceGetMapData(nav_msgs::GetMap::Response& map_, const hectorslam::GridMap& gridMap);

  void publishTransformLoop(double p_transform_pub_period_);

  void publishMapLoop(double p_map_pub_period_);

  /*=========================control thread ================================*/
  //void AnHung_Control(double anhung_control_period);
  void commandCallback(const hector_mapping::setmap_hec& command);

  /*=========================Socket timeout ================================*/
  int socket_recvtimeout(int ss, char *buf, int len, int timeout);

  /*===========================LoadTitlePath ================================*/
  void LoadTitlePath();
  /*===========================Socket error ================================*/
  void diep(char *s);

  /*===========================Create Map ==================================*/
  void CreateMap();

  /*============================Load Map ===================================*/
  void LoadMap(std::string Map_Name);
  void LoadChangeMap(std::string Map_Name,Eigen::Vector3f ini_pose);
  void LoadChangeMap_early(std::string Map_Name);

  /*============================save Map ===================================*/
  void SaveMap(std::string Save_Name);

  /*==============================int to string ============================*/
  std::string int2str(int i);

  /*==============================float to string ==========================*/
  std::string float2str(float i);


  /*============================check how many lasers ======================*/
  int checkDeviceNum(const sensor_msgs::LaserScan& scan);


  //=========================Get Relocation Pose ==========================//
  void RePoseCallback(const geometry_msgs::PoseStamped& msg);

  void publishTransform();

  void staticMapCallback(const nav_msgs::OccupancyGrid& map);

  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);


  /*==============================Get Navigation pose ======================*/
  void goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);


  //========================Receive odometry speed ======================//
  void OdomSpeedCallback(const geometry_msgs::PoseStamped& msg);

  void NormalizeAngle(float& phi);
  void automappingForChangeMap(Eigen::Vector3f ini_pose);
  void SetLocalpar(std::string file_buf);


  sensor_msgs::LaserScan device_scan_1;
  sensor_msgs::LaserScan device_scan_2;
  int device_num;


  /*
  void setStaticMapData(const nav_msgs::OccupancyGrid& map);
  */
protected:

  HectorDebugInfoProvider* debugInfoProvider;
  HectorDrawings* hectorDrawings;

  int lastGetMapUpdateIndex;

  ros::NodeHandle node_;

  ros::Subscriber scanSubscriber_;
  ros::Subscriber sysMsgSubscriber_;


  //==============================Get Command ==============================//
  ros::Subscriber commandSubscriber_;


  ros::Subscriber mapSubscriber_;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>* initial_pose_sub_;
  tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>* initial_pose_filter_;


  //==============================Get Navigation pose ======================//
  message_filters::Subscriber<geometry_msgs::PoseStamped>* goal_pose_sub_;
  tf::MessageFilter<geometry_msgs::PoseStamped>* goal_pose_filter_;



  ros::Publisher posePublisher_;
  ros::Publisher poseUpdatePublisher_;
  ros::Publisher twistUpdatePublisher_;
  ros::Publisher odometryPublisher_;
  ros::Publisher scan_point_cloud_publisher_;

  //=========================Get Relocation Pose ==========================//
  ros::Subscriber Relocation_pose_sub_;

  //==============================Publish Goal pose ======================//
  ros::Publisher goal_publisher_;

  //=========================Publish missing signal ======================//
  ros::Publisher miss_publisher_;
    //=====error=========================================================//
  ros::Publisher Error_Publisher_;
  //=====Publish change finish=========================================================//
  ros::Publisher floor_Publisher_;
  ros::Publisher Mapname_Publisher_;


  std::vector<MapPublisherContainer> mapPubContainer;

  tf::TransformListener tf_;
  tf::TransformBroadcaster* tfB_;



  //========================================================================//
  //laser_geometry::LaserProjection projector_;
  laser_geometry::LaserProjection projector_1_;
  laser_geometry::LaserProjection projector_2_;

  tf::Transform map_to_odom_;

  boost::thread* map__publish_thread_;
  boost::thread* LoadChangeMap_early_thread_;
  /*=====Anhung====================control thread ================================*/
  //boost::thread* anhung_control_thread_;

  hectorslam::HectorSlamProcessor* slamProcessor;
  hectorslam::HectorSlamProcessor* early_load;


  /*=========================Two Laser =====================================*/
  //hectorslam::DataContainer laserScanContainer;
  hectorslam::DataContainer laserScanContainer_1;
  hectorslam::DataContainer laserScanContainer_2;

  PoseInfoContainer poseInfoContainer_;


  /*=========================Two Laser =====================================*/
  //sensor_msgs::PointCloud laser_point_cloud_;
  sensor_msgs::PointCloud laser_point_cloud_1_;
  sensor_msgs::PointCloud laser_point_cloud_2_;




  ros::Time lastMapPublishTime;
  ros::Time lastScanTime;
  Eigen::Vector3f lastSlamPose;

  bool initial_pose_set_;
  Eigen::Vector3f initial_pose_;


  //-----------------------------------------------------------
  // Parameters


  /*=========================Socket parameter ==============================*/
  /*
  struct sockaddr_in p_si_me_, p_si_other_;
  socklen_t p_slen_;
  char p_socket_buf_[P_SOCKET_BUFLEN];
  int p_s_socket_;
  */
  /*=========================bool navigation ==============================*/
  bool p_navigation_;
  bool p_Relocation_;

  std::string p_base_frame_;
  std::string p_map_frame_;
  std::string p_odom_frame_;

  /*==========================control state ===============================*/
  int p_control_state_;


  ros::Publisher scan_data_publisher_;
  ros::Publisher scan_data_publisher_2;



  //=========================Receive odometry speed ======================//
  ros::Subscriber OdomSpeedSubscriber_;
  //============================odometry speed==========================//
  float odom_vx;
  float odom_vy;
  float odom_w;
  //==========================odometry predict==========================//
  bool isReceiveOdom;
  bool isInitialTimeSample;
  double p_time;




  //Parameters related to publishing the scanmatcher pose directly via tf
  bool p_pub_map_scanmatch_transform_;
  std::string p_tf_map_scanmatch_transform_frame_name_;

  std::string p_scan_topic_;
  std::string p_sys_msg_topic_;

  std::string p_pose_update_topic_;
  std::string p_twist_update_topic_;

  bool p_pub_drawings;
  bool p_pub_debug_output_;
  bool p_pub_map_odom_transform_;
  bool p_pub_odometry_;
  bool p_advertise_map_service_;
  int p_scan_subscriber_queue_size_;

  double p_update_factor_free_;
  double p_update_factor_occupied_;
  double p_map_update_distance_threshold_;
  double p_map_update_angle_threshold_;

  double p_map_resolution_;
  int p_map_size_;
  double p_map_start_x_;
  double p_map_start_y_;
  int p_map_multi_res_levels_;

  double p_map_pub_period_;

  bool p_use_tf_scan_transformation_;
  bool p_use_tf_pose_start_estimate_;
  bool p_map_with_known_poses_;
  bool p_timing_output_;


  float p_sqr_laser_min_dist_;
  float p_sqr_laser_max_dist_;
  float p_laser_z_min_value_;
  float p_laser_z_max_value_;

    //protect
  std::string protect_map_type;
  std::string protect_map_name;
//PATH
std::string TitlePath;
std::string LocalparPATH;
std::string map_name;

bool loadfinish;
};


#endif
