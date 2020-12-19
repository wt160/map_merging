/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Tang Wei.   tangwei6268@hotmail.com
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
 *   * Neither the name of the Tang Wei nor the names of its
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



#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <random>


std::vector<nav_msgs::OccupancyGrid> global_map_list;


std::vector<std::pair<double, double>> getMapCorners(double input_origin_x, double input_origin_y, double input_width, double input_height){
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


void mapCallback(const nav_msgs::OccupancyGridConstPtr& map, int id){
    // ROS_WARN_STREAM("mapCallback: id="<<id);
    if(global_map_list.size() < id){
        return;
    }
    if(global_map_list.size() == id){
        global_map_list.push_back(*map);
    }else{
        global_map_list[id] = *map;
    }
}



double matchingCostOfMapList(std::vector<nav_msgs::OccupancyGrid>& map_list, std::vector<std::pair<double, double>> robot_pose_list){
    //get the maximal corners containing all the map
    
    double input_origin_x = map_list[0].info.origin.position.x;
    double input_origin_y = map_list[0].info.origin.position.y;
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
    
    for(int i = 0; i < map_list.size(); i++){
        double curr_origin_x = map_list[i].info.origin.position.x;
        double curr_origin_y = map_list[i].info.origin.position.y;
        double curr_width = map_list[i].info.width * map_list[i].info.resolution;
        double curr_height = map_list[i].info.height * map_list[i].info.resolution;
        double from_world_to_curr_x = robot_pose_list[i].first;
        double from_world_to_curr_y = robot_pose_list[i].second;

        std::vector<std::pair<double, double>> curr_corners = getMapCorners(curr_origin_x + from_world_to_curr_x, curr_origin_y + from_world_to_curr_y, curr_width, curr_height);
        



        for(auto i = curr_corners.begin(); i != curr_corners.end(); i++){
            
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





    }
    // ROS_INFO("2");
    nav_msgs::OccupancyGrid output;
    output.info = map_list[0].info;
    output.header = map_list[0].header;
    output.data = map_list[0].data;

    //merge all the map under the frame of /world
    double output_origin_x = x_extre;
    double output_origin_y = y_extre;
    int output_width_cell = (int)((x_max - x_min) / output.info.resolution);
    int output_height_cell = (int)((y_max - y_min) / output.info.resolution); 


    output.info.width = output_width_cell;
    output.info.height = output_height_cell;

    output.info.origin.position.x = output_origin_x;
    output.info.origin.position.y = output_origin_y;



    //during merge, calculate the matchingcost = sum of all the cells value, which equals the product of value from all the map, if value == -1. treat it as 0
    output.data.clear();
    std::vector<double> output_data_vector;
    output_data_vector.resize(output_width_cell * output_height_cell, 1);
    output.data.resize(output_width_cell * output_height_cell, 1);
    for(int i=  0; i < map_list.size(); i++){
        double from_world_to_curr_x = robot_pose_list[i].first;
        double from_world_to_curr_y = robot_pose_list[i].second;
        double target_origin_x_in_world_frame = (from_world_to_curr_x + map_list[i].info.origin.position.x);
        double target_origin_y_in_world_frame = (from_world_to_curr_y + map_list[i].info.origin.position.y);
        int target_origin_to_output_origin_x_cell_in_input_frame = (output_origin_x - target_origin_x_in_world_frame) / map_list[i].info.resolution;    //output_origin_x_cell - input_origin_x_cell
        int target_origin_to_output_origin_y_cell_in_input_frame = (output_origin_y - target_origin_y_in_world_frame) / map_list[i].info.resolution;    //output_origin_y_cell - input_origin_y_cell
        for(int x = 0; x < map_list[i].info.width; x++){
            for(int y = 0; y < map_list[i].info.height; y++){
                int new_x = x - target_origin_to_output_origin_x_cell_in_input_frame;
                int new_y = y - target_origin_to_output_origin_y_cell_in_input_frame;
                int target_value = map_list[i].data[y*map_list[i].info.width + x];
                //could add more checking to compare the value from both input map and from target map
                // ROS_WARN_STREAM("target_value:"<<target_value);
                if(target_value < 0){      
                    output_data_vector[new_y*output_width_cell + new_x] = 0.0;
                }else if(target_value < 5){
                    output_data_vector[new_y*output_width_cell + new_x] = 0.0;
                    
                }else{ 
                    int previous_value = output_data_vector[new_y*output_width_cell + new_x];
                    //ROS_WARN_STREAM("previous value:"<<(double)previous_value);
                    output_data_vector[new_y*output_width_cell + new_x] = (double)target_value * (double)previous_value;
                }
                if((double)output_data_vector[new_y*output_width_cell + new_x] < 0){
                    ROS_ERROR_STREAM("something strange happened "<<(double)output_data_vector[new_y*output_width_cell + new_x]);
                }


                // if(previous_value < OBSTACLE_COST){ 
                //     if(target_value > OBSTACLE_COST){
                //         output.data[new_y*output_width_cell + new_x] = target_value;
                //     }else if(target_value != -1){
                //         if(previous_value == -1){
                //             output.data[new_y*output_width_cell + new_x] = target_value;
                //         }else{
                //             if(target_value < previous_value){
                //                 output.data[new_y*output_width_cell + new_x] = target_value;
                //             }
                //         }
                        
                //     }
                // }
            }
        }


    }
    // ROS_INFO_STREAM("output_data_vector.size:"<<output_data_vector.size()<<" , output size:"<<output.info.width * output.info.height);
    double final_cost = 0;
    for(int x = 0; x < output.info.width; x++){
        for(int y = 0; y < output.info.height; y++){
            //ROS_WARN_STREAM("value:"<<(double)output.data[y*output.info.width + x]);

            final_cost += (double)output_data_vector[y*output.info.width + x];
        }
    }
    return final_cost;

}



std::vector<std::vector<std::pair<double, double>>> getNeighborSamples(std::vector<std::pair<double, double>> sample, double step_size){
    std::vector<std::vector<std::pair<double, double>>> neighbor_list;
    for(int s = 0; s < 2; s ++){ 
        for(int i = 0; i < sample.size() * 2; i++){
            std::vector<std::pair<double, double>> neigh = sample;
            if(i % 2 == 0){ 
                neigh[i / 2].first += step_size;
            }else{
                neigh[i / 2].second += step_size;
            }
            neighbor_list.push_back(neigh);
        }
        step_size = - step_size;
    }
    return neighbor_list;

}



double hillClimb(std::vector<nav_msgs::OccupancyGrid>& map_list, std::vector<std::pair<double, double>> robot_init_pose, 
                    std::vector<std::pair<double, double>>& local_optima_pose, double step_size, std::map<double, bool> candidate_optima_cost_map){
    double init_cost = matchingCostOfMapList(map_list, robot_init_pose);
    bool is_reach_optima = false;
    int trial_num = 30;
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    while((!is_reach_optima) || trial_num > 0 ){
        std::vector<std::vector<std::pair<double, double>>> neighbors = getNeighborSamples(robot_init_pose, step_size);
        double max_cost = std::numeric_limits<double>::min();
        std::vector<std::pair<double, double>> candidate_next_pose;
        for(int i = 0; i < neighbors.size(); i++){
            // ROS_INFO("1");
            double cost = matchingCostOfMapList(map_list, neighbors[i]);
            if(cost > max_cost){
                max_cost = cost;
                candidate_next_pose = neighbors[i];
            }
        }
        ROS_WARN_STREAM("max:"<<max_cost<<" ,init:"<<init_cost);
        if(candidate_optima_cost_map.find(max_cost) != candidate_optima_cost_map.end()){
            local_optima_pose = candidate_next_pose;
            return max_cost;
        }
        if(max_cost > init_cost){
            robot_init_pose = candidate_next_pose;
            init_cost = max_cost;
        }else{
            is_reach_optima = true;
            trial_num --;
            int next_index = floor((int)dist(mt)*neighbors.size());
            robot_init_pose = neighbors[next_index];
        }
        ROS_WARN_STREAM("trial_num:"<<trial_num);
    }
    local_optima_pose = robot_init_pose;
    return init_cost;
}


// multi-sample + local hill climbing
bool calibrateRobotInitPose(std::vector<nav_msgs::OccupancyGrid>& map_list, std::vector<std::pair<double, double>> robot_init_pose, 
                                 double sample_radius, int sample_num, double step_size,
                                 std::vector<std::pair<double, double>>& updated_robot_pose_list, std::map<double, bool>& candidate_optima_cost_map,
                                 std::map<std::vector<std::pair<double, double>>, bool>& candidate_optima_robot_pose_map){
    // multi-sample around the init robot pose
    std::vector<std::vector<std::pair<double, double>>> sample_init_pose_list;
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    for(int s = 0; s < sample_num; s++){
        std::vector<std::pair<double, double>> new_sample_list;
        double r = 0.0;
        double theta = 0.0;
        for(auto i = robot_init_pose.begin(); i != robot_init_pose.end(); i++){
            r = sample_radius * dist(mt); 
            theta = dist(mt)* 2* 3.14159;
            std::pair<double, double> new_sample;
            new_sample.first = i->first + r * cos(theta);
            new_sample.second = i->second + r*sin(theta);
            new_sample_list.push_back(new_sample);
        }
    
        sample_init_pose_list.push_back(new_sample_list); 
    }

    std::vector<std::pair<double, double>> right_init_pose;
    right_init_pose.push_back(std::make_pair(0, 5));
    right_init_pose.push_back(std::make_pair(0, 3));
    right_init_pose.push_back(std::make_pair(0, 1));
    right_init_pose.push_back(std::make_pair(0, -1));
    right_init_pose.push_back(std::make_pair(0, -3));
    double right_cost = matchingCostOfMapList(map_list, right_init_pose);

    ROS_WARN_STREAM("Done sampling, right cost:"<<right_cost);



    // for each sample , local hill climbing to find the local optima, then compare to get the final optima result
    double global_optima_cost = std::numeric_limits<double>::min();
    int thread_num = 10;
    std::vector<double> discovered_cost_list;
    std::vector<std::pair<double, double>> sample, optima_neighbor;
    
    for(int i = 0; i < sample_init_pose_list.size(); i++){
        sample = sample_init_pose_list[i];
        
        
        double optima_cost = hillClimb(map_list ,sample, optima_neighbor, step_size, candidate_optima_cost_map);
        

        if(candidate_optima_cost_map.find(optima_cost) == candidate_optima_cost_map.end()){
            candidate_optima_cost_map[optima_cost] = true;
            ROS_WARN_STREAM("adding optima cost "<<optima_cost);

        }
        if(candidate_optima_robot_pose_map.find(optima_neighbor) == candidate_optima_robot_pose_map.end()){
            candidate_optima_robot_pose_map[optima_neighbor] = true;
            ROS_WARN_STREAM("adding optima neighbor:  ");
            for(int x = 0; x < optima_neighbor.size(); x ++){
                ROS_WARN_STREAM("robot "<<x<<" pose: ("<<optima_neighbor[x].first<<","<<optima_neighbor[x].second<<")");
            }
        }


        if(optima_cost > global_optima_cost){
            discovered_cost_list.push_back(optima_cost);
            global_optima_cost = optima_cost;
            updated_robot_pose_list = optima_neighbor;
        }
        ROS_WARN_STREAM("checking "<<i<<" sample, local_cost="<<optima_cost<<" global_cost="<<global_optima_cost);   
        

    }

    return true;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "robot_init_pose_calibration");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh_("~");

    int robot_num = 0;
    std::string robot_init_pose_string, robot_name_list_string;
    std::vector<std::pair<double, double>> robot_init_pose;
    int sample_num = 0;
    double sample_radius = 0.0;
    double step_size = 0.01;

    //get params
    private_nh_.param<int>("robot_num", robot_num, 5);
    private_nh_.param<std::string>("robot_name_list", robot_name_list_string, "");
    private_nh_.param<std::string>("robot_init_pose", robot_init_pose_string, "");
    private_nh_.param<int>("sample_num", sample_num, 10000);
    private_nh_.param<double>("sample_radius", sample_radius, 1.0);
    private_nh_.param<double>("step_size", step_size, 0.01);

    if(robot_init_pose_string.size() == 0 || robot_name_list_string.size() == 0){
        ROS_FATAL_STREAM("Raw robot init pose or robot name list not given, exit...");
        exit(0);
    }

    //parse the input robot init pose string and name string
    

    std::istringstream iss(robot_init_pose_string);
    std::vector<std::string> robot_init_pose_list((std::istream_iterator<std::string>(iss)),
                                 std::istream_iterator<std::string>());

    std::istringstream iss2(robot_name_list_string);
    std::vector<std::string> robot_name_list((std::istream_iterator<std::string>(iss2)),
                                 std::istream_iterator<std::string>());
    for(int i = 0; i < robot_num*2; i+= 2){
        std::pair<double, double> pose;
        pose.first = std::stod(robot_init_pose_list[i]);
        pose.second = std::stod(robot_init_pose_list[i+1]);
        robot_init_pose.push_back(pose);
    }



    std::vector<ros::Subscriber> map_sub_list;
    
    for(int i = 0; i < robot_num; i ++){  
        std::string robot_map_name = "/costmap_calibration_" + robot_name_list[i] + "/costmap/costmap";
        map_sub_list.push_back(nh.subscribe<nav_msgs::OccupancyGrid>(robot_map_name, 1, boost::bind(mapCallback, _1, i)));
        ros::spinOnce();
    }
    std::vector<std::pair<double, double>> updated_robot_pose_list;
    while(global_map_list.size() < robot_num){
        ros::spinOnce();
        // ROS_WARN_STREAM("global map size:"<<global_map_list.size());
    }

    std::map<double, bool> candidate_optima_cost_map;
    std::map<std::vector<std::pair<double, double>>, bool> candidate_optima_robot_pose_map;
    calibrateRobotInitPose(global_map_list, robot_init_pose, sample_radius, sample_num, step_size, updated_robot_pose_list, candidate_optima_cost_map, candidate_optima_robot_pose_map);

    //print the updated robot pose list
    for(int i = 0; i < updated_robot_pose_list.size(); i++){
        ROS_INFO_STREAM("Robot "<<i<<" init pose:"<<"("<<updated_robot_pose_list[i].first<<","<<updated_robot_pose_list[i].second<<")");
    }

    ros::spin();
}