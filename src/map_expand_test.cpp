#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>


ros::Publisher map_pub;
nav_msgs::OccupancyGrid input_map;
nav_msgs::OccupancyGrid target_map;
nav_msgs::OccupancyGrid output_map;



#define OBSTACLE_COST 55
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

/*
*   input_pose and target_pose, these poses need to be in the same frame, no matter what frame. Now only supports same orientation
*   output map shares the same frame as the input map
*   TODO: add support for maps with orientation difference
*/
void map_expand(nav_msgs::OccupancyGrid& input, nav_msgs::OccupancyGrid& target, nav_msgs::OccupancyGrid & output, 
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



void target_map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    target_map.data = msg->data;
    target_map.info = msg->info;
    target_map.header = msg->header;

}


void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    geometry_msgs::Pose input_pose;
    geometry_msgs::Pose target_pose;
    input_pose.position.x = 0;
    input_pose.position.y = 5;
    input_pose.position.z = 0.0;
    input_pose.orientation.x = 0;
    input_pose.orientation.y = 0;
    input_pose.orientation.z = 0;
    input_pose.orientation.w = 1;
    target_pose.position.x = 0;
    target_pose.position.y = -3;
    target_pose.position.z = 0.0;
    target_pose.orientation.x = 0;
    target_pose.orientation.y = 0;
    target_pose.orientation.z = 0;
    target_pose.orientation.w = 1;
    input_map.data = msg->data;
    input_map.info = msg->info;
    input_map.header = msg->header;


    map_expand(input_map, target_map, output_map, input_pose, target_pose);

    map_pub.publish(output_map);
    ros::spinOnce();
    ROS_WARN_STREAM("publish expand map");

}




int main(int argc, char** argv){
    ros::init(argc, argv, "robot_pose_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh_("~");

    ros::Subscriber sub_2 = nh.subscribe("/X1_4/map", 1, target_map_cb);
    ros::Subscriber sub = nh.subscribe("/X1_0/map", 1, map_cb);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("X1_0/expand_map", 10);
    // private_nh_.param("",);
    ros::spin();
}