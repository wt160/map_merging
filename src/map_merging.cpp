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

#include "map_merging.hpp"

MapMerging::MapMerging() {
  ros::NodeHandle private_nh("~");
  private_nh.param<double>("merging_rate", merging_rate_, 10.0);
  private_nh.param<int>("max_number_robots", max_number_robots_, 100);
  private_nh.param<double>("max_comm_distance", max_comm_distance_, 10.0);
  private_nh.param<std::string>("pose_topic", pose_topic_, "pose");
  private_nh.param<std::string>("map_topic", map_topic_, "map");
  private_nh.param<std::string>("merged_map_topic", merged_map_topic_, "merged_map");
  private_nh.param<double>("init_pose_x", init_x_, 0);
  private_nh.param<double>("init_pose_y", init_y_, 0);
  private_nh.param<double>("init_pose_z", init_z_, 0);
  private_nh.param<double>("init_pose_yaw", init_yaw_, 0);
  private_nh.param<std::string>("world_frame", world_frame_, "world");
  private_nh.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");
  geometry_msgs::Pose init_pose;
  init_pose.position.x = init_x_;
  init_pose.position.y = init_y_;
  init_pose.position.z = init_z_;
  tf::Quaternion init_pose_quaternion;
  init_pose_quaternion.setEuler(init_yaw_, 0, 0);
  init_pose.orientation.x = init_pose_quaternion.x();
  init_pose.orientation.y = init_pose_quaternion.y();
  init_pose.orientation.z = init_pose_quaternion.z();
  init_pose.orientation.w = init_pose_quaternion.w();

  


  /* publisher params */
  map_has_been_merged_ = new bool[max_number_robots_]();
  merged_map_publisher_ = private_nh.advertise<nav_msgs::OccupancyGrid>(merged_map_topic_, 1, true);
  
  /* get robot's name */
  // my_name_ = ros::this_node::getNamespace(); // Get the robot's name.
  my_name_ = robot_base_frame_;
  // my_name_.erase(0, 1); // [ROS_BUG #3671] Remove the first slash in front of the name.
  
  curr_pose_publisher_ = node_.advertise<geometry_msgs::PoseStamped>( pose_topic_, 1, true);
  my_id_ = tm_id_ = UNKNOWN;
}

MapMerging::~MapMerging() {
  delete[] map_has_been_merged_;
}

/*
 * topicSubscribing()
 */
void MapMerging::topicSubscribing() {
  ros::master::V_TopicInfo topic_infos;
  ros::master::getTopics(topic_infos);
  
  for(ros::master::V_TopicInfo::const_iterator it_topic = topic_infos.begin(); it_topic != topic_infos.end(); ++it_topic) {
    const ros::master::TopicInfo &published_topic = *it_topic;
    /* robot pose subscribing */
    if(published_topic.name.find(pose_topic_) != std::string::npos && !poseFound(published_topic.name, poses_)) {
      ROS_INFO("[%s]:Subscribe to POSE topic: %s.", (ros::this_node::getName()).c_str(), published_topic.name.c_str());
      poses_.push_back(Pose(published_topic.name, false));
      pose_subsribers_.push_back(node_.subscribe<geometry_msgs::PoseStamped>(published_topic.name, 1, boost::bind(&MapMerging::poseCallback, this, _1, poses_.size()-1)));
      /* get robot's ID */
      ROS_WARN_STREAM("my_name_:"<<my_name_<<"     published_topic:"<<published_topic.name);
      if(my_name_.size() > 0 && published_topic.name.compare(1, my_name_.size(), my_name_) == 0) {
	      my_id_ = poses_.size()-1;
	      ROS_INFO("[%s]:My name is %s, ID = %d.", (ros::this_node::getName()).c_str(), my_name_.c_str(), my_id_);
      }
    }
  }
  
  if(my_id_ == UNKNOWN) {
    ROS_WARN("[%s]:Can not get robot's pose.", (ros::this_node::getName()).c_str());
  }
}

/*
 * handShaking(): handshaking if robots are within the communication range.
 */
void MapMerging::handShaking() {
  if(my_id_ == UNKNOWN || tm_id_ != UNKNOWN)
    return;
  
  geometry_msgs::Pose my_pose, tm_pose;
  if(poses_[my_id_].received) {
    my_pose.position.x = poses_[my_id_].data.pose.position.x;
    my_pose.position.y = poses_[my_id_].data.pose.position.y;
    my_pose.position.z = poses_[my_id_].data.pose.position.z;
    poses_[my_id_].received = false; 
    //std::cout << "[" << ros::this_node::getName() << "] " << poses_[my_id_].data.header.frame_id << " x = " << my_pose.position.x << " y = " << my_pose.position.y << std::endl;
    tm_id_list_.clear();
    for(int i = 0; i < (int)poses_.size(); i++) {
      if(i != my_id_ && !map_has_been_merged_[i]) {
        tm_name_ = robotNameParsing(poses_[i].name);
        if(poses_[i].received) {
          tm_pose.position.x = poses_[i].data.pose.position.x;
          tm_pose.position.y = poses_[i].data.pose.position.y;
          tm_pose.position.z = poses_[i].data.pose.position.z;
          poses_[i].received = false;
          if(distBetween(tm_pose.position, my_pose.position) < max_comm_distance_) {
            ROS_DEBUG("[%s]:Handshake with %s.", (ros::this_node::getName()).c_str(), tm_name_.c_str());
            tm_id_ = i;
            tm_id_list_.push_back(tm_id_);
          }
        }
      }
    }
  }
  
  if(tm_id_ == UNKNOWN) {
    // do something here ...
  }
}

/*
 * mapMerging()
 */
void MapMerging::mapMerging() {
  if(my_id_ == UNKNOWN || tm_id_ == UNKNOWN)
    return;
  
  std::string map_topic_name;
  for(int i = 0; i < (int)poses_.size(); i++) {
    map_topic_name = robotNameParsing(poses_[i].name)+"/"+map_topic_;
    if(!mapFound(map_topic_name, maps_)) {
      maps_.push_back(Map(map_topic_name, false));
      map_subsribers_.push_back(node_.subscribe<nav_msgs::OccupancyGrid>(map_topic_name, 1, boost::bind(&MapMerging::mapCallback, this, _1, maps_.size()-1)));
      ROS_INFO("[%s]:Subscribe to MAP topic: %s.", (ros::this_node::getName()).c_str(), map_topic_name.c_str());
    }
  }
  
  merged_map_ = maps_[my_id_].data;
  
  bool map_merging_end = false;
  for(int t = 0; t < tm_id_list_.size(); t++){
    tm_id_ = tm_id_list_[t];
    tm_name_ = robotNameParsing(poses_[tm_id_].name);
    if(maps_[my_id_].received) {
      if(maps_[tm_id_].received) {
        ROS_WARN("[%s]:Exchange map with %s.", (ros::this_node::getName()).c_str(), tm_name_.c_str());
    //std::cout << "[" << ros::this_node::getName() << "] deltaX = " << delta.position.x << " deltaY = " << delta.position.y << std::endl;
        //if(!map_has_been_merged_[my_id_]) {
        //  merged_map_ = maps_[my_id_].data;
        //  map_has_been_merged_[my_id_] = true;
        //}
        nav_msgs::OccupancyGrid temp_output;
        geometry_msgs::Pose my_init_pose, tm_init_pose;
        if(!getInitPose(my_name_, my_init_pose)){
          ROS_ERROR_STREAM("Could not get init pose for "<<my_name_);
        }

        if(!getInitPose(tm_name_, tm_init_pose)){
          ROS_ERROR_STREAM("Could not get init pose for "<<tm_name_);
        }

        map_expand(merged_map_, maps_[tm_id_].data, temp_output, 
                my_init_pose, tm_init_pose);
        maps_[my_id_].data.info = temp_output.info;
        maps_[my_id_].data.header = temp_output.header;
        maps_[my_id_].data.data = temp_output.data;
        merged_map_ = maps_[my_id_].data;

        //GreedyMerging(round(delta.position.x), round(delta.position.y), tm_id_);	  
        maps_[tm_id_].received = false;
      }
    }
  }
  maps_[my_id_].received = false;
  map_merging_end = true;
  
  if(map_merging_end) {
    ROS_WARN("[%s]:Map has been merged with %s.", (ros::this_node::getName()).c_str(), tm_name_.c_str());
    map_has_been_merged_[tm_id_] = true;
    tm_id_ = UNKNOWN;
    
    // It has coordinated with all that can be coordinated, so reset.
    bool reset_and_publish = true;
    // for(int i = 0; i < (int)poses_.size(); i++) {
    //   if(!map_has_been_merged_[i]) {
	  //     reset_and_publish = false;
	  //     break;
    //   }
    // }
    
    if(reset_and_publish) {
      for(int i = 0; i < (int)poses_.size(); i++)
	      map_has_been_merged_[i] = false;
      merged_map_publisher_.publish(maps_[my_id_].data);
    }
  }
}

/*********************
 * callback function *
 *********************/
void MapMerging::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int id) {
  ROS_DEBUG("poseCallback");
  //boost::mutex::scoped_lock pose_lock (pose_mutex_);
  poses_[id].received = true;
  poses_[id].data = *msg;
}

void MapMerging::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg, int id) {
  ROS_DEBUG("mapCallback");
  //boost::mutex::scoped_lock map_lock (map_mutex_);
  maps_[id].received = true;
  maps_[id].data = *msg;
}

/*********************
 * private function *
 *********************/
bool MapMerging::poseFound(std::string name, std::vector<Pose> &poses) {
  for(std::vector<Pose>::iterator it_pose = poses.begin(); it_pose != poses.end(); ++it_pose) {
    if((*it_pose).name.compare(name) == 0)
      return true;
  }
  return false;
}

bool MapMerging::mapFound(std::string name, std::vector<Map> &maps) {
  for(std::vector<Map>::iterator it_map = maps.begin(); it_map != maps.end(); ++it_map) {
    if((*it_map).name.compare(name) == 0)
      return true;
  }
  return false;
}

std::string MapMerging::robotNameParsing(std::string s) {
  return s.erase(s.find('/', 1), s.size());
}

/* 
 * Get robot's initial position
 * TODO: get orientation
 */
bool MapMerging::getInitPose(std::string name, geometry_msgs::Pose &pose) {
  if(ros::param::get("/"+name+"/map_merge/init_pose_x", pose.position.x) &&
     ros::param::get("/"+name+"/map_merge/init_pose_y", pose.position.y) &&
     ros::param::get("/"+name+"/map_merge/init_pose_z", pose.position.z)) {
    return true;
  }
  return false;
}

/* 
 * Get the relative position of two robots
 * TODO: get orientation
 */
bool MapMerging::getRelativePose(std::string n, std::string m, geometry_msgs::Pose &delta, double resolution) {
  geometry_msgs::Pose p, q;
  if(ros::param::get(n+"/map_merge/init_pose_x", p.position.x) &&
     ros::param::get(n+"/map_merge/init_pose_y", p.position.y) &&
     ros::param::get(n+"/map_merge/init_pose_z", p.position.z) &&
     ros::param::get(m+"/map_merge/init_pose_x", q.position.x) &&
     ros::param::get(m+"/map_merge/init_pose_y", q.position.y) &&
     ros::param::get(m+"/map_merge/init_pose_z", q.position.z)) {
    delta.position.x = round((p.position.x - q.position.x) / resolution);
    delta.position.y = round((p.position.y - q.position.y) / resolution);
    delta.position.z = round((p.position.z - q.position.z) / resolution);
    return true;
  }
  return false;
}

double MapMerging::distBetween(geometry_msgs::Point &p, geometry_msgs::Point &q) {
  // Euclidean distance
  return sqrt((p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y)+(p.z-q.z)*(p.z-q.z));
}

/*
 * Algorithm 1 - Greedy Merging
 * yz14simpar
 */
void MapMerging::greedyMerging(int delta_x, int delta_y, int map_id) {
  for(int i = 0; i < (int)merged_map_.info.width; i++) {
    for(int j = 0; j < (int)merged_map_.info.height; j++) {
      if(i+delta_x >= 0 && i+delta_x < (int)maps_[map_id].data.info.width &&
	 j+delta_y >= 0 && j+delta_y < (int)maps_[map_id].data.info.height) {
	if((int)merged_map_.data[i+j*(int)merged_map_.info.width] == UNKNOWN)
	  merged_map_.data[i+j*(int)merged_map_.info.width] = maps_[map_id].data.data[i+delta_x+(j+delta_y)*(int)maps_[map_id].data.info.width];
      }
    }
  }
}



/*
*   input_pose and target_pose, these poses need to be in the same frame, no matter what frame. Now only supports same orientation
*   output map shares the same frame as the input map
*   TODO: add support for maps with orientation difference
*/
void MapMerging::map_expand(nav_msgs::OccupancyGrid& input, nav_msgs::OccupancyGrid& target, nav_msgs::OccupancyGrid & output, 
                 geometry_msgs::Pose input_pose, geometry_msgs::Pose target_pose){
    output.info = input.info;
    output.header = input.header;
    output.data = input.data;
    double from_input_to_target_x = target_pose.position.x - input_pose.position.x;
    double from_input_to_target_y = target_pose.position.y - input_pose.position.y;
    int from_input_to_target_x_cell = from_input_to_target_x / input.info.resolution;
    int from_input_to_target_y_cell = from_input_to_target_y / input.info.resolution;

    //first expand the input map, resize to new size, that will contain both the input and the target map




    double input_width = input.info.width * input.info.resolution;
    double input_height = input.info.height * input.info.resolution;
    double target_width = target.info.width * target.info.resolution;
    double target_height = target.info.height * target.info.resolution;

    

    

    std::vector<std::pair<double, double>> input_corners;
    std::vector<std::pair<double, double>> target_corners;

    double input_origin_x = input.info.origin.position.x;
    double input_origin_y = input.info.origin.position.y;
    double target_origin_x = target.info.origin.position.x;
    double target_origin_y = target.info.origin.position.y;


    
    input_corners = getMapCorners(input_origin_x, input_origin_y, input_width, input_height);
    //target_corners in the input map frame
    target_corners = getMapCorners(target_origin_x + from_input_to_target_x, target_origin_y + from_input_to_target_y, target_width, target_height);
    
    bool is_input_origin_x_min = false;
    bool is_input_origin_y_min = false;
    if(input_origin_x > 0 && input_origin_y > 0){
        is_input_origin_x_min = false;
        is_input_origin_y_min = false;
    }else if(input_origin_x > 0 && input_origin_y < 0){
        is_input_origin_x_min = false;
        is_input_origin_y_min = true;
    }else if(input_origin_x < 0 && input_origin_y > 0){
        is_input_origin_x_min = true;
        is_input_origin_y_min = false;
    }else if(input_origin_x < 0 && input_origin_y < 0){
        is_input_origin_x_min = true;
        is_input_origin_y_min = true;
    }

    double output_origin_x = 0.0;
    double output_origin_y = 0.0;

    double x_max = 0.0;
    double x_min = 0.0;
    double y_max = 0.0;
    double y_min = 0.0;
    double x_extre = 0.0;
    double y_extre = 0.0;
    if(is_input_origin_x_min){
        x_extre = std::numeric_limits<double>::max();
    }else{
        x_extre = std::numeric_limits<double>::min();
    }
    if(is_input_origin_y_min){
        y_extre = std::numeric_limits<double>::max();
    }else{
        y_extre = std::numeric_limits<double>::min();
    }
    x_max = std::numeric_limits<double>::min();
    x_min = std::numeric_limits<double>::max();
    y_max = std::numeric_limits<double>::min();
    y_min = std::numeric_limits<double>::max();



    for(auto i = input_corners.begin(); i != input_corners.end(); i++){
        
        if(i->first >= x_max){
            x_max = i->first;
        }
        if(i->first < x_min){
            x_min = i->first;
        }
        if(i->second >= y_max){
            y_max = i->second;
        }
        if(i->second < y_min){
            y_min = i->second;
        }
        
        
        
        if(is_input_origin_x_min){
            if(i->first < x_extre){
                x_extre = i->first;
            }        
        }else{
            if(i->first > x_extre){
                x_extre = i->first;
            }
        }
        if(is_input_origin_y_min){
            if(i->second < y_extre){
                y_extre = i->second;
            }        
        }else{
            if(i->second > y_extre){
                y_extre = i->second;
            }
        }
    }

    for(auto i = target_corners.begin(); i != target_corners.end(); i++){
        if(i->first >= x_max){
            x_max = i->first;
        }
        if(i->first < x_min){
            x_min = i->first;
        }
        if(i->second >= y_max){
            y_max = i->second;
        }
        if(i->second < y_min){
            y_min = i->second;
        }
        
        
        if(is_input_origin_x_min){
            if(i->first < x_extre){
                x_extre = i->first;
            }        
        }else{
            if(i->first > x_extre){
                x_extre = i->first;
            }
        }
        
        if(is_input_origin_y_min){
            if(i->second < y_extre){
                y_extre = i->second;
            }        
        }else{
            if(i->second > y_extre){
                y_extre = i->second;
            }
        }

    }
    output_origin_x = x_extre;
    output_origin_y = y_extre;
    int output_width_cell = (int)((x_max - x_min) / input.info.resolution);
    int output_height_cell = (int)((y_max - y_min) / input.info.resolution); 


    output.info.width = output_width_cell;
    output.info.height = output_height_cell;

    output.info.origin.position.x = output_origin_x;
    output.info.origin.position.y = output_origin_y;


    //second fill the region of the original input map region
    output.data.clear();
    output.data.resize(output_width_cell * output_height_cell, -1);
    int input_origin_to_output_origin_x_cell = (int)((output_origin_x - input.info.origin.position.x) / input.info.resolution);    //output_origin_x_cell - input_origin_x_cell
    int input_origin_to_output_origin_y_cell = (int)((output_origin_y - input.info.origin.position.y) / input.info.resolution);    //output_origin_y_cell - input_origin_y_cell
    for(int x = 0; x < input.info.width; x++){
        for(int y = 0; y < input.info.height; y++){
            int new_x = x - input_origin_to_output_origin_x_cell;
            int new_y = y - input_origin_to_output_origin_y_cell;
            int input_value = input.data[y*input.info.width + x];
            output.data[new_y*output_width_cell + new_x] = input_value;
        }
    }
    //last, fill or merge the region belonging to the target map
    double target_origin_x_in_input_frame = (from_input_to_target_x + target.info.origin.position.x);
    double target_origin_y_in_input_frame = (from_input_to_target_y + target.info.origin.position.y);
    int target_origin_to_output_origin_x_cell_in_input_frame = (output_origin_x - target_origin_x_in_input_frame) / input.info.resolution;    //output_origin_x_cell - input_origin_x_cell
    int target_origin_to_output_origin_y_cell_in_input_frame = (output_origin_y - target_origin_y_in_input_frame) / input.info.resolution;    //output_origin_y_cell - input_origin_y_cell
    for(int x = 0; x < target.info.width; x++){
        for(int y = 0; y < target.info.height; y++){
            int new_x = x - target_origin_to_output_origin_x_cell_in_input_frame;
            int new_y = y - target_origin_to_output_origin_y_cell_in_input_frame;
            int target_value = target.data[y*target.info.width + x];
            //could add more checking to compare the value from both input map and from target map
            
            int previous_value = output.data[new_y*output_width_cell + new_x];
            if(previous_value < OBSTACLE_COST){ 
                if(target_value > OBSTACLE_COST){
                    output.data[new_y*output_width_cell + new_x] = target_value;
                }else if(target_value != -1){
                    if(previous_value == -1){
                        output.data[new_y*output_width_cell + new_x] = target_value;
                    }else{
                        if(target_value < previous_value){
                            output.data[new_y*output_width_cell + new_x] = target_value;
                        }
                    }
                    
                }
            }
        }
    }


}


std::vector<std::pair<double, double>> MapMerging::getMapCorners(double input_origin_x, double input_origin_y, double input_width, double input_height){
    std::vector<std::pair<double, double>> input_corners;
    input_corners.push_back(std::make_pair(input_origin_x, input_origin_y));
    if(input_origin_x > 0 && input_origin_y > 0){ 
        input_corners.push_back(std::make_pair(input_origin_x - input_width, input_origin_y));
        input_corners.push_back(std::make_pair(input_origin_x - input_width, input_origin_y - input_height));
        input_corners.push_back(std::make_pair(input_origin_x, input_origin_y - input_height));
    
    }else if(input_origin_x > 0 && input_origin_y < 0){
        input_corners.push_back(std::make_pair(input_origin_x - input_width, input_origin_y));
        input_corners.push_back(std::make_pair(input_origin_x - input_width, input_origin_y + input_height));
        input_corners.push_back(std::make_pair(input_origin_x, input_origin_y + input_height));
    
    }else if(input_origin_x < 0 && input_origin_y > 0){
        input_corners.push_back(std::make_pair(input_origin_x + input_width, input_origin_y));
        input_corners.push_back(std::make_pair(input_origin_x + input_width, input_origin_y - input_height));
        input_corners.push_back(std::make_pair(input_origin_x, input_origin_y - input_height));
    
    }else if(input_origin_x < 0 && input_origin_y < 0){
        input_corners.push_back(std::make_pair(input_origin_x + input_width, input_origin_y));
        input_corners.push_back(std::make_pair(input_origin_x + input_width, input_origin_y + input_height));
        input_corners.push_back(std::make_pair(input_origin_x, input_origin_y + input_height));
    
    }
    return input_corners;
}


void MapMerging::posePublishing(){
  tf::TransformListener listener(node_);
  geometry_msgs::PoseStamped curr_pose;
  
  tf::StampedTransform transform;
  try
  {
      listener.waitForTransform(world_frame_, robot_base_frame_, ros::Time(0), ros::Duration(30.0));
      listener.lookupTransform(world_frame_, robot_base_frame_, ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
      ROS_ERROR("TF %s - %s: %s", world_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
  }
  curr_pose.pose.position.x = transform.getOrigin().getX();
  curr_pose.pose.position.y = transform.getOrigin().getY();
  curr_pose.pose.position.z = transform.getOrigin().getZ();
  curr_pose.pose.orientation.x = transform.getRotation().getX();
  curr_pose.pose.orientation.y = transform.getRotation().getY();
  curr_pose.pose.orientation.z = transform.getRotation().getZ();
  curr_pose_publisher_.publish(curr_pose);
  ros::spinOnce();
}

/*
 * execute()
 */
void MapMerging::execute() {
  ros::Rate r(merging_rate_);
  while(node_.ok()) {
    topicSubscribing();
    handShaking();
    mapMerging();
    posePublishing();
    r.sleep();
  }
}

/*
 * spin()
 */
void MapMerging::spin() {
  ros::spinOnce();
  boost::thread t(boost::bind(&MapMerging::execute, this));
  ros::spin();
  t.join();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_merging");
  MapMerging map_merging;
  map_merging.spin();
  return 0;
}
