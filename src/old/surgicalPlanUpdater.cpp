/*
 * Surgical Plan Updater
 *
 * Hipster (c) 2022-2023
 * author: Gunjan Sethi, gunjans@andrew.cmu.edu
 *         Kaushik Balasundar, kaushik@cmu.edu
 * 
 * subscribe to pelvis pose
 * Publish pelvis -> initial_end_point tansform
 * subscribe to error
 * if err = true:
 *  lookup pelvis -> initial_end_point tansform
 *  new reaming ep = current_pelvis pose * pelvis->initial_ep
 *     
 */

// Include libraries for ROS
#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>

using namespace std;

// Checks if error = true and performs 
void updateReamingEndPoint( const std_msgs::BoolConstPtr& error, 
                            ros::Publisher* reaming_end_point_publisher,
                            geometry_msgs::PoseStamped* currentPelvisPose){
    if(error->data == true){
        geometry_msgs::TransformStamped pelvisReamingReamingPointTransform;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        try{
            // ros::Duration(0.5).sleep();
            pelvisReamingReamingPointTransform = tfBuffer.lookupTransform("pelvis", "initial_reaming_end_point", ros::Time(0));
            geometry_msgs::PoseStamped updatedReamingEndPoint; // = tfListener.doTransform()
            tf2::doTransform(currentPelvisPose, updatedReamingEndPoint, pelvisReamingReamingPointTransform);
            reaming_end_point_publisher->publish(updatedReamingEndPoint);
            

            }
        

        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

}

void reaming_end_point_publisher(const geometry_msgs::PoseStampedConstPtr& error,
                                 ros::Publisher* reaming_end_point_publisher){

}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "surgicalPlanUpdater");
    ros::NodeHandle n;
    ros::Rate loop_rate(60);

    geometry_msgs::PoseStamped currentPelvisPose;

    ros::Publisher reaming_end_point_publisher = n.advertise<geometry_msgs::PoseStamped>("reaming_end_point", 1000);
    ros::Subscriber pelvis_pose_subscriber = n.subscribe<geometry_msgs::PoseStamped>("pelvis_pose", 1000, bind(callback, _1, &currentPelvisPose)); 
    ros::Subscriber error_subscriber = n.subscribe<std_msgs::Bool>("pelvis_error", 1000, bind(updateReamingEndPoint, _1, &reaming_end_point_publisher, &currentPelvisPose));

    // Calculate (pelvis -> end_point) transform
    tf2::Transform pelvis_pose;
    tf2::Transform initial_reaming_end_point;
    tf2::Transform pelvisReamingPointTransform = calculatePelvisReamingPointTransform(pelvis_pose, initial_reaming_end_point);
    

    while (ros::ok()){
        
    }

    return 0;
}