#include <map>
#include "ros/ros.h"
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"
using namespace std;

extern ros::NodeHandle frameHandlerNodeHandler;

struct markerInfo{
    string geomFilePath;
    string body;
    string frame;
    string parent_frame;
    ros::Publisher pub;
    tf2_ros::TransformBroadcaster br;
    string topic_name;
};

extern std::map<int, markerInfo> markerMap;

// Pose Publishers
extern ros::Publisher probe_pose_publisher = frameHandlerNodeHandler.advertise<geometry_msgs::PoseStamped>("probe_pose", 1000);
extern ros::Publisher pelvis_pose_publisher = frameHandlerNodeHandler.advertise<geometry_msgs::PoseStamped>("pelvis_pose", 1000);
extern ros::Publisher end_effector_pose_publisher = frameHandlerNodeHandler.advertise<geometry_msgs::PoseStamped>("end_effector_pose", 1000);

// Broadcasters
extern tf2_ros::TransformBroadcaster probe_tranform_broadcaster;
extern tf2_ros::TransformBroadcaster pelvis_tranform_broadcaster;
extern tf2_ros::TransformBroadcaster end_effector_tranform_broadcaster;

extern struct markerInfo probe_marker = {
"/home/mrsd-team-c/arthur_ws/src/arthur_perception/data/geometry8888.ini",
"Registration Probe",
"probe",
"camera",
probe_pose_publisher,
probe_tranform_broadcaster,
"probe_pose"
};

extern struct markerInfo pelvis_marker = {
    "/home/mrsd-team-c/arthur_ws/src/arthur_perception/data/geometry9990.ini",
    "Pelvis",
    "pelvis",
    "camera",
    pelvis_pose_publisher,
    pelvis_tranform_broadcaster,
    "pelvis_pose"
};