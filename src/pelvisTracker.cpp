/*
 * Pelvis Tracker
 *
 * Hipster (c) 2022-2023
 * author: Gunjan Sethi, gunjans@andrew.cmu.edu
 *         Kaushik Balasundar, kaushik@cmu.edu
 *     
 */
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <eigen3/Eigen/Dense>

bool DEBUG = true;

using namespace std;

float calculatePositionError(geometry_msgs::Point initialPosition
                            ,geometry_msgs::Point currentPosition){
    float positionError;
    positionError = sqrt(pow((initialPosition.x - currentPosition.x), 2) +
                pow((initialPosition.y - currentPosition.y), 2) +
                pow((initialPosition.z - currentPosition.z), 2));
    
    return positionError;
}

tf::Matrix3x3 convertToRotation(tf::Quaternion quat){
    tf::Matrix3x3 rot;

    rot = tf::Matrix3x3(quat);

    return rot;
}

vector<double> calculateRotError(tf::Matrix3x3 initialOrientation, tf::Matrix3x3 currentOrientation){

    vector<double> err(3);
    tf::Matrix3x3 rotError = currentOrientation * initialOrientation.inverse();
    rotError.getRPY(err[0], err[1], err[2]);
     
    return err;
}


void detectError(const geometry_msgs::PoseStampedConstPtr& currentPelvisPose
                ,ros::Publisher* error_publisher ){
    std_msgs::Float64MultiArray errors;

    static geometry_msgs::PoseStamped initialPose = *currentPelvisPose;

    // Calculate Pose Error
    float positionError = calculatePositionError( initialPose.pose.position
                                                , currentPelvisPose->pose.position);

    tf::Quaternion initialPoseQuat(initialPose.pose.orientation.x,
                                       initialPose.pose.orientation.y,
                                       initialPose.pose.orientation.z,
                                       initialPose.pose.orientation.w);

    tf::Quaternion currentPoseQuat(currentPelvisPose->pose.orientation.x,
                                       currentPelvisPose->pose.orientation.y,
                                       currentPelvisPose->pose.orientation.z,
                                       currentPelvisPose->pose.orientation.w);

    tf::Matrix3x3 initialOrientationRPY = convertToRotation(initialPoseQuat);
    tf::Matrix3x3 currentOrientationRPY = convertToRotation(currentPoseQuat);
    vector<double> orientationErrorInDegrees = calculateRotError(initialOrientationRPY
                                                ,currentOrientationRPY);

    if(DEBUG){
    cout << "Pelvis : positionError : " << positionError << " "; 
        cout << "Pelvis : RPYError : " << orientationErrorInDegrees[0] << " " 
                                                << orientationErrorInDegrees[1] << " "
                                                << orientationErrorInDegrees[2] << " " <<endl; 
    }

    std_msgs::Bool error;
    if(positionError >= 0.003 or orientationErrorInDegrees[0] >= 0.0523599 or orientationErrorInDegrees[1] >= 0.0523599 or orientationErrorInDegrees[2] >= 0.0523599){
        cout << "Error Detected!\n";
        if(positionError >= 0.003){
            // cout << "Pelvis : positionError : " << positionError << " "; 
        }
        else{
            // cout << "Pelvis : RPYError : " << orientationErrorInDegrees[0] << " " 
                                            // << orientationErrorInDegrees[1] << " "
                                            // << orientationErrorInDegrees[2] << " " <<endl; 
        }
        error.data = true;
        error_publisher->publish(error);
        initialPose = *currentPelvisPose;
        return;
    }
    error.data = false;
    error_publisher->publish(error);        
    // return 0;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "pelvisTracker");
    ros::NodeHandle n;
    ros::Publisher error_publisher = n.advertise<std_msgs::Bool>("pelvis_error", 1000);
    ros::Subscriber pelvis_pose_sub = n.subscribe<geometry_msgs::PoseStamped>("pelvis_pose", 1000, bind(detectError, _1, &error_publisher)); 
    ros::spin();
    return 0;
}