/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Zhi Yan.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Zhi Yan nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#ifndef _MAP_MERGING_HPP
#define _MAP_MERGING_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <tf/transform_listener.h>

static const int UNKNOWN = -1;
static const int OBSTACLE = 100;
static const int FREE = 0;
#define OBSTACLE_COST 55

struct Pose {
  std::string name;
  bool received;
  geometry_msgs::PoseStamped data;
  
  Pose() : name(), received(), data() {}
  Pose(std::string n, bool r) : name(n), received(r), data() {}
};

struct Map {
  std::string name;
  bool received;
  nav_msgs::OccupancyGrid data;
  
  Map() : name(), received(), data() {}
  Map(std::string n, bool r) : name(n), received(r), data() {}
};

class MapMerging {
private:
  ros::NodeHandle node_;
  
  /*** ROS parameters ***/
  double merging_rate_;
  int max_number_robots_;
  double max_comm_distance_;
  double init_x_, init_y_, init_z_, init_yaw_;
  std::string pose_topic_, map_topic_, merged_map_topic_;
  std::string world_frame_, robot_base_frame_;


  /*** ROS publishers ***/
  bool *map_has_been_merged_;
  nav_msgs::OccupancyGrid merged_map_;
  ros::Publisher merged_map_publisher_;
  ros::Publisher curr_pose_publisher_;

  /*** ROS subsribers ***/
  std::vector<Pose> poses_;
  std::vector<ros::Subscriber> pose_subsribers_;
  std::vector<Map> maps_;
  std::vector<ros::Subscriber> map_subsribers_;
  //boost::mutex pose_mutex_, map_mutex_;
  
  std::string my_name_, tm_name_;
  int my_id_, tm_id_;
  std::vector<int> tm_id_list_;
  
  bool poseFound(std::string name, std::vector<Pose> &poses);
  bool mapFound(std::string name, std::vector<Map> &maps);
  
  std::string robotNameParsing(std::string s);
  bool getInitPose(std::string name, geometry_msgs::Pose &pose);
  bool getRelativePose(std::string n, std::string m, geometry_msgs::Pose &delta, double resolution);
  double distBetween(geometry_msgs::Point &p, geometry_msgs::Point &q);
  
  /*** Map merging algorithms ***/
  void greedyMerging(int delta_x, int delta_y, int map_id);
  void map_expand(nav_msgs::OccupancyGrid& input, nav_msgs::OccupancyGrid& target, nav_msgs::OccupancyGrid & output, 
                 geometry_msgs::Pose input_pose, geometry_msgs::Pose target_pose);

  std::vector<std::pair<double, double>> getMapCorners(double input_origin_x, double input_origin_y, double input_width, double input_height);


public:
  MapMerging();
  ~MapMerging();
  
  void spin();
  void execute();
  
  void topicSubscribing();
  void handShaking();
  void mapMerging();
  void posePublishing();
  
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int id);
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg, int id);
};

#endif /* !_MAP_MERGING_HPP */
