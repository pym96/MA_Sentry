#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>

std::string odom_topic = "state_estimation";
std::string path_topic = "move_base1/NavfnROS/plan";
float distance_thre = 1.2;
float distance_tole = 0.6;

float vehicle_x = 0, vehicle_y = 0, vehicle_z = 0;
double cur_time = 0.0, way_point_time = 0.0;
double way_point_temp_x = 0.0;
double way_point_temp_y = 0.0;

bool flag_get_new_path;
bool flag_finish_path;
bool flag_switch_goal;

std::vector<geometry_msgs::PoseStamped> way_point_array;

void poseHandler(const nav_msgs::Odometry::ConstPtr& pose){
    cur_time = pose->header.stamp.toSec();
    
    vehicle_x = pose->pose.pose.position.x;
    vehicle_y = pose->pose.pose.position.y;
    vehicle_z = pose->pose.pose.position.z;
}

   /**
     * Get path into array 
     * Choose the nearest point(longer than 1 meter)
    */
void pathHandler(const nav_msgs::Path::ConstPtr& path){
    if(!path->poses.empty()){
        flag_get_new_path = true;
        way_point_array.clear();
        
        std::vector<geometry_msgs::PoseStamped> path_array = path->poses;
        // Catch the first position
        geometry_msgs::PoseStamped first_pos = *path_array.begin();
        // Catch the last position
        geometry_msgs::PoseStamped last_pos = *(path_array.end() - 1);
        float distance_2d = 0;
        double temp_x = vehicle_x;
        double temp_y = vehicle_y;
        
        for(const auto& pose_stamped : path_array){
            distance_2d = sqrt(pow(pose_stamped.pose.position.x - temp_x, 2) + 
                               pow(pose_stamped.pose.position.y - temp_y, 2));

            if(distance_2d > distance_thre){
                way_point_array.push_back(pose_stamped);
                temp_x = pose_stamped.pose.position.x;
                temp_y = pose_stamped.pose.position.y;
                ROS_INFO("choose point: (%f %f)", temp_x, temp_y);
            }
        }
        way_point_array.push_back(last_pos);
    }   

}

int main(int argc, char** argv){

    ros::init(argc, argv, "path2waypoint");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("odom_topic", odom_topic);
    nhPrivate.getParam("path_topic", path_topic);
    nhPrivate.getParam("distance_thre", distance_thre);

    ros::Subscriber sub_pose = nh.subscribe<nav_msgs::Odometry> (odom_topic, 5, poseHandler);
    ros::Subscriber sub_path = nh.subscribe<nav_msgs::Path> (path_topic, 5, pathHandler);
    
    ros::Publisher pub_way_point = nh.advertise<geometry_msgs::PointStamped> ("/way_point", 5);
    geometry_msgs::PointStamped way_point_msgs;
    way_point_msgs.header.frame_id = "map";;

    geometry_msgs::PoseStamped goal_point;
    int path_index;
    double goal_point_distance;
    
    ros::Rate rate(100);
    bool statue = ros::ok();
    while(statue){

        ros::spinOnce();
        
        if(!way_point_array.empty()){
            if(flag_get_new_path){
                goal_point = way_point_array.front();
                path_index = 0;
                flag_switch_goal = true;
                flag_get_new_path = false;
            }else{
                goal_point_distance = sqrt(pow(goal_point.pose.position.x - vehicle_x, 2)
                                           + pow(goal_point.pose.position.y - vehicle_y, 2));
                
                if((goal_point_distance < distance_tole) && (path_index < way_point_array.size() - 1))
                {
                    path_index++;
                    goal_point = way_point_array.at(path_index);
                    flag_switch_goal = true;
                }
                else if(path_index == way_point_array.size() - 1){
                    flag_finish_path = true;
                }

            }
        }
           
           if(flag_switch_goal){
            way_point_msgs.header.stamp = ros::Time().fromSec(cur_time);
            way_point_msgs.point.x = goal_point.pose.position.x;
            way_point_msgs.point.y = goal_point.pose.position.y;
            way_point_msgs.point.z = 0.0;
            pub_way_point.publish(way_point_msgs);
           }

           statue = ros::ok();
           rate.sleep();
    }

    return 0;

}