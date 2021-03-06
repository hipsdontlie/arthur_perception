/*
 * Class Name: pointcloudCollector
 *
 * Team Name: Hipster (c) 2022-2023
 * Author(s): Gunjan Sethi, gunjans@andrew.cmu.edu
 *         Kaushik Balasundar, kaushikb@cmu.edu
 * 
 * Team Members: Kaushik Balasundar, Anthony Kyu, Gunjan Sethi, Sundaram Seivur, Parker Hill
 * 
 * High Level Description: Collects an arbitrary number of points from the registration probe, converts to PointCloud2 type and publishes as a topic    
 * 
 * Date of first revision: 17th March 2022      
 */


// Include libraries for ROS
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Header.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <deque>
#include <iomanip>
#include <iostream>
#include "helpers.hpp"
#include <sstream>

#include "geometryHelper.hpp"

#include <signal.h>

using namespace std;

// Pointcloud Collection Hyperparams
int num_points = 3000;
int points_added = 0;
int time_elapsed = 2;
ros::Time publish_time;

static volatile bool keep_going = true;

// Create publisher for pointcloud, populate message with points from marker 

int collectPelvisPointcloud(ros::NodeHandle n){

    signal (SIGINT, [](int) {keep_going = false;});

    ros::Publisher pelvis_pointcloud_publisher = n.advertise<sensor_msgs::PointCloud2>("pelvis_pointcloud", 1000);
    static int points_collected = 0;

    sensor_msgs::PointCloud2 pelvis_cloud;
    sensor_msgs::PointCloud2Modifier modifier(pelvis_cloud);
    
    modifier.resize(num_points);
    modifier.setPointCloud2Fields(
        3, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
        sensor_msgs::PointField::FLOAT32, "z", 1,
        sensor_msgs::PointField::FLOAT32);

    sensor_msgs::PointCloud2Iterator<float> iter_x(pelvis_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pelvis_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pelvis_cloud, "z");

    geometry_msgs::TransformStamped camera_probetip_transform;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    //int flag = -1;
    while (points_collected < num_points){
        
        
        try{
            if (tfBuffer.canTransform("camera", "probetip", ros::Time::now(), ros::Duration(3.0)) == true){
                cout << "Collecting pointcloud!" << endl;
                camera_probetip_transform = tfBuffer.lookupTransform("camera", "probetip", ros::Time(0));
                ++ points_collected;
                // ros::Duration(0.1).sleep();
                cout << "Total Points Collected: "<< points_collected << endl;

                *iter_x = camera_probetip_transform.transform.translation.x;
                *iter_y = camera_probetip_transform.transform.translation.y;
                *iter_z = camera_probetip_transform.transform.translation.z;
            
                ++iter_x;
                ++iter_y;
                ++iter_z;

                pelvis_cloud.header.frame_id = "camera";
                pelvis_cloud.header.stamp = ros::Time::now();
                pelvis_cloud.height = 1;
                pelvis_cloud.width = num_points;
                pelvis_cloud.is_dense = false;

                pelvis_pointcloud_publisher.publish(pelvis_cloud);

            }

            else {ROS_INFO("Probe not Visible!");}


     
            }
        

        catch (tf2::TransformException &ex) {

            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    cout<<"\nThanks for collecting points! Saving your pointcloud :)\n";
    n.setParam("save_pointcloud", true);

    return points_collected;
}


//main function 

int main(int argc, char **argv){
    
    ros::init(argc, argv, "pointcloudCollector");
    ros::NodeHandle n;
    ros::Rate loop_rate(60);
    collectPelvisPointcloud(n);
    
    return 0;
}