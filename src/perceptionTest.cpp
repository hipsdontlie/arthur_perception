/*
 * Class Name: PercpetionTest 
 *
 * Team Name: Hipster (c) 2022-2023
 * Author(s): Gunjan Sethi, gunjans@andrew.cmu.edu
 *         Kaushik Balasundar, kaushikb@cmu.edu
 * 
 * Team Members: Kaushik Balasundar, Anthony Kyu, Gunjan Sethi, Sundaram Seivur, Parker Hill
 * High Level Description: Error detection accuracy test for SVD

 * Date of first revision: 7th April 2022  
 */

// Include libraries for ROS
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <eigen3/Eigen/Dense>
#include <chrono>

using namespace std;

//Calculate position error 

float calculatePositionError(geometry_msgs::Point initialPosition
                            ,geometry_msgs::Point currentPosition){
    float positionError;
    positionError = sqrt(pow((initialPosition.x - currentPosition.x), 2) +
                pow((initialPosition.y - currentPosition.y), 2) +
                pow((initialPosition.z - currentPosition.z), 2));
    
    return positionError;
}

//Convert quaternion to Euler angles

Eigen::Vector3f convertToRPY(Eigen::Quaternionf quat){
    Eigen::Vector3f rpy;
    double roll, pitch, yaw;
    rpy = Eigen::Matrix3f(quat).eulerAngles(0,1,2);
    return rpy;
}

// compute error in RPY
Eigen::Vector3f calculateRPYError(Eigen::Vector3f initialOrientation, Eigen::Vector3f currentOrientation){

    Eigen::Vector3f rpyError = (180*(initialOrientation - currentOrientation))/3.14;
     
    return rpyError;
}


//get reference marker 1 pose 

void getMarkerPose1(const geometry_msgs::PoseStampedConstPtr& marker1Pose
                    , geometry_msgs::PoseStamped* marker1_pose){
    
    marker1_pose->header = marker1Pose->header;
    marker1_pose->pose = marker1Pose->pose;
                
}

// get reference marker 2 pose 

void getMarkerPose2(const geometry_msgs::PoseStampedConstPtr& marker2Pose
                    , geometry_msgs::PoseStamped* marker2_pose){

    marker2_pose->header = marker2Pose->header;
    marker2_pose->pose = marker2Pose->pose;

}        

//main function 
int main(int argc, char **argv){
    
    ros::init(argc, argv, "perceptionTest");
    ros::NodeHandle n;
        
    geometry_msgs::PoseStamped marker1_pose;
    geometry_msgs::PoseStamped marker2_pose;

    ros::Subscriber marker_test1_sub = n.subscribe<geometry_msgs::PoseStamped>("test_marker1_pose", 1000, bind(getMarkerPose1, _1, &marker1_pose)); 
    ros::Subscriber marker_test2_sub = n.subscribe<geometry_msgs::PoseStamped>("test_marker2_pose", 1000, bind(getMarkerPose2, _1, &marker2_pose)); 

    auto start = std::chrono::steady_clock::now();

    float poseError = calculatePositionError(marker1_pose.pose.position, marker2_pose.pose.position);
    Eigen::Vector3f marker1_or = convertToRPY(Eigen::Quaternionf(marker1_pose.pose.orientation.x,
                                                                 marker1_pose.pose.orientation.y,
                                                                 marker1_pose.pose.orientation.z,
                                                                 marker1_pose.pose.orientation.w));
    Eigen::Vector3f marker2_or = convertToRPY(Eigen::Quaternionf(marker2_pose.pose.orientation.x,
                                                                 marker2_pose.pose.orientation.y,
                                                                 marker2_pose.pose.orientation.z,
                                                                 marker2_pose.pose.orientation.w));
    Eigen::Vector3f orError = calculateRPYError(marker1_or, marker2_or);

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;

    cout << "poseError (mm) : " << poseError << endl;
    cout << "orError (degrees) : " << orError[0] << " : " << orError[1] << " : " << orError[2] << endl;
    cout << "Elapsed time (s) : " << elapsed_seconds.count() << "s\n";
    cout << "---------" << endl;

    ros::spin();
    return 0;
}