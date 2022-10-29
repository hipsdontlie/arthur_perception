/*
 * Class Name: Frame Handler
 *
 * Team Name: Hipster (c) 2022-2023
 * Author(s): Gunjan Sethi, gunjans@andrew.cmu.edu
 *            Kaushik Balasundar, kaushikb@cmu.edu
 * Team Members: Kaushik Balasundar, Anthony Kyu, Gunjan Sethi, Sundaram Seivur, Parker Hill

* High Level Description: 
    Reads marker data from Atracsys SDK and publishes the poses as ROS message types 

Date of first revision: 1st February 2022    
 */

// Include libraries for ROS
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Header.h"
#include <eigen3/Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>

// Include libraries for camera
#include <algorithm>
#include <deque>
#include <iomanip>
#include <iostream>
#include "helpers.hpp"
#include <sstream>
#include <map>

#include <bits/stdc++.h>

#include "geometryHelper.hpp"

// #include "markerInfo.h"
#include <ros/console.h>

using namespace std;

bool TEST_MODE = false;

struct euler_representation{
    float64 r00, r01, r02;
    float64 r10, r11, r12;
    float64 r20, r21, r22;
};

struct markerInfo{
    string geomFilePath;
    string body;
    string frame;
    string parent_frame;
    ros::Publisher pub;
    tf2_ros::TransformBroadcaster br;
    string topic_name;
};

std::unordered_map<int, markerInfo> markerMap;

// Camera variables
static const unsigned ENABLE_ONBOARD_PROCESSING_OPTION = 6000;
static const unsigned SENDING_IMAGES_OPTION = 6003;

ftkLibrary lib = 0;
uint64 sn( 0uLL );
// Configuration file
string cfgFile( "" );

/*
Function to initialize camera driver. Returns NULL if
it fails to initialize.
*/
ftkLibrary initializeCameraDriver(string cfgFile){
    ftkBuffer buffer;
    ftkLibrary lib = ftkInitExt( cfgFile.empty() ? nullptr : cfgFile.c_str(), &buffer );
    if ( lib == nullptr )
    {
        cerr << buffer.data << endl;
        // error( "Cannot initialize driver" , !isNotFromConsole );
        return nullptr;
    }
    return lib;
}

/*
Function to discover data about the connected camera.
*/
DeviceData discoverCamera(ftkLibrary lib){
    DeviceData device(retrieveLastDevice(lib));
    return device;
}

/*
TypeCaste and Publish frame
*/
int publishPoses(ftkMarker* marker
                , tf2_ros::TransformBroadcaster br
                , string body
                , ros::Publisher* pub
                , string parent_frame
                , string child_frame){

    
    // Declare new PoseStamped message
    geometry_msgs::PoseStamped marker_pose_msg;   

    // Declare new TransformStamped message
    geometry_msgs::TransformStamped marker_transform;

    // Initialize transform frame header
    ros::Time stamp = ros::Time::now(); 

    marker_transform.header.stamp = stamp;
    marker_transform.header.frame_id = parent_frame;
    marker_transform.child_frame_id = child_frame;

    marker_pose_msg.header.stamp = stamp;
    marker_pose_msg.header.frame_id = parent_frame;

    euler_representation er;
    er.r00 = marker->rotation[ 0u ][ 0u ];
    er.r01 = marker->rotation[ 0u ][ 1u ];
    er.r02 = marker->rotation[ 0u ][ 2u ];
    er.r10 = marker->rotation[ 1u ][ 0u ];
    er.r11 = marker->rotation[ 1u ][ 1u ];
    er.r12 = marker->rotation[ 1u ][ 2u ];
    er.r20 = marker->rotation[ 2u ][ 0u ];
    er.r21 = marker->rotation[ 2u ][ 1u ];
    er.r22 = marker->rotation[ 2u ][ 2u ];

    Eigen::Matrix3f m;
    m <<er.r00, er.r01, er.r02,
        er.r10, er.r11, er.r12,
        er.r20, er.r21, er.r22;
    Eigen::Quaternionf q(m);

    marker_transform.transform.translation.x = marker_pose_msg.pose.position.x = marker->translationMM[ 0u ]/1000.00;
    marker_transform.transform.translation.y = marker_pose_msg.pose.position.y = marker->translationMM[ 1u ]/1000.00;
    marker_transform.transform.translation.z = marker_pose_msg.pose.position.z = marker->translationMM[ 2u ]/1000.00;

    marker_transform.transform.rotation.x = marker_pose_msg.pose.orientation.x = q.x();
    marker_transform.transform.rotation.y = marker_pose_msg.pose.orientation.y = q.y();
    marker_transform.transform.rotation.z = marker_pose_msg.pose.orientation.z = q.z();
    marker_transform.transform.rotation.w = marker_pose_msg.pose.orientation.w = q.w();

    // Publish
    pub->publish(marker_pose_msg);
    // Broadcast
    br.sendTransform(marker_transform);

    if(body == "Registration Probe"){

        geometry_msgs::TransformStamped probetip_transform;


        // ros::Time stamp = ros::Time::now(); 
        probetip_transform.header.stamp = stamp;
        probetip_transform.header.frame_id = "probe";
        probetip_transform.child_frame_id = "probetip";  

        probetip_transform.transform.translation.x = 0 ;
        probetip_transform.transform.translation.y = 288.6583 / 1000 ;
        probetip_transform.transform.translation.z = 8.07974 / 1000 ;

        probetip_transform.transform.rotation.x = 0; 
        probetip_transform.transform.rotation.y = 0; 
        probetip_transform.transform.rotation.z = 0; 
        probetip_transform.transform.rotation.w = 1;

        br.sendTransform(probetip_transform);


    }

    return 0;
}


int main(int argc, char **argv)
{
    // ------------------------------------------------------------------------

    ROS_INFO("Activating Frame Handler ..");
        
    // ROS Initialization
     ros::init(argc, argv, "frameHandler");
     ros::NodeHandle n;
     ros::Rate loop_rate(60);

    //  Pose Publishers
     ros::Publisher probe_pose_publisher = n.advertise<geometry_msgs::PoseStamped>("probe_pose", 1000);
     ros::Publisher pelvis_pose_publisher = n.advertise<geometry_msgs::PoseStamped>("pelvis_pose", 1000);
     ros::Publisher end_effector_pose_publisher = n.advertise<geometry_msgs::PoseStamped>("end_effector_pose", 1000);
     ros::Publisher probe_tip_pose_publisher = n.advertise<geometry_msgs::PoseStamped>("probe_tip_pose", 1000);
     
     
    //  Broadcasters
     tf2_ros::TransformBroadcaster probe_tranform_broadcaster;
     tf2_ros::TransformBroadcaster pelvis_tranform_broadcaster;
     tf2_ros::TransformBroadcaster end_effector_tranform_broadcaster;
     tf2_ros::TransformBroadcaster probe_tip_tranform_broadcaster;

    struct markerInfo probe_marker = {
        "/home/mrsd-team-c/arthur_ws/src/arthur_perception/data/geometry8888.ini",
        "Registration Probe",
        "probe",
        "camera",
        probe_pose_publisher,
        probe_tranform_broadcaster,
        "probe_pose"
        };


    struct markerInfo pelvis_marker = {
        "/home/mrsd-team-c/arthur_ws/src/arthur_perception/data/geometry9990.ini",
        "Pelvis",
        "pelvis",
        "camera",
        pelvis_pose_publisher,
        pelvis_tranform_broadcaster,
        "pelvis_pose"
    };


    struct markerInfo probe_tip_marker = {
        "/home/mrsd-team-c/arthur_ws/src/arthur_perception/data/geometry8888.ini",
        "Probe Tip",
        "probetip",
        "camera",
        probe_tip_pose_publisher,
        probe_tip_tranform_broadcaster,
        "probe_tip_pose"
    };


    struct markerInfo ee_marker = {
        "/home/mrsd-team-c/arthur_ws/src/arthur_perception/data/geometry24090.ini",
        "End Effector Marker",
        "ee_marker",
        "camera",
        end_effector_pose_publisher,
        end_effector_tranform_broadcaster,
        "ee_pose"
    };


    markerMap.insert(std::pair<int, markerInfo>(8888, probe_marker));
    markerMap.insert(std::pair<int, markerInfo>(9990, pelvis_marker));
    markerMap.insert(std::pair<int, markerInfo>(24090, ee_marker));
    markerMap.insert(std::pair<int, markerInfo>(1264, probe_tip_marker));




    if(TEST_MODE){
        ros::Publisher test_marker1_publisher = n.advertise<geometry_msgs::PoseStamped>("test_marker1_pose", 1000);
        ros::Publisher test_marker2_publisher = n.advertise<geometry_msgs::PoseStamped>("test_marker2_pose", 1000);
        tf2_ros::TransformBroadcaster test_marker1_broadcaster;
        tf2_ros::TransformBroadcaster test_marker2_broadcaster;
        struct markerInfo test_marker1 = {
            "/home/mrsd-team-c/arthur_ws/src/arthur_perception/data/geometry9876.ini",
            "Test Marker 1",
            "test_marker1",
            "camera",
            test_marker1_publisher,
            test_marker1_broadcaster,
            "test_marker1_pose"
        };

        struct markerInfo test_marker2 = {
            "/home/mrsd-team-c/arthur_ws/src/arthur_perception/data/geometry9876.ini",
            "Test Marker 2",
            "test_marker2",
            "camera",
            test_marker2_publisher,
            test_marker2_broadcaster,
            "test_marker2_pose"
        };
        markerMap.insert(std::pair<int, markerInfo>(9876, test_marker1));
        markerMap.insert(std::pair<int, markerInfo>(9875, test_marker2));
    }


    // ------------------------------------------------------------------------

    // ------------------------------------------------------------------------
    
    // Camera Initialization
    lib = initializeCameraDriver(cfgFile); 
    if(lib==nullptr){
        cout << "Failed to initialize driver";
        return 1;
    }
    // Discover camera
    DeviceData camera = discoverCamera(lib);
    uint64 sn( camera.SerialNumber );

    // Set current temperature index
    ftkSetInt32( lib, sn, 30, 1 );   

    if (ftkDeviceType::DEV_SPRYTRACK_180 == camera.Type)
    {
        cout << "Enable onboard processing" << endl;
        if ( ftkSetInt32( lib, sn, ENABLE_ONBOARD_PROCESSING_OPTION, 1 ) != ftkError::FTK_OK )
        {
            error( "Cannot process data directly on the SpryTrack.");
        }

        cout << "Disable images sending" << endl;
        if ( ftkSetInt32( lib, sn, SENDING_IMAGES_OPTION, 0 ) != ftkError::FTK_OK )
        {
            error( "Cannot disable images sending on the SpryTrack."  );
        }
    }

    // ------------------------------------------------------------------------

    // ------------------------------------------------------------------------

    // Set geometry

    // Geometry file
    deque< string > geomFiles{ 
        markerMap[8888].geomFilePath,
        markerMap[9990].geomFilePath,
        markerMap[24090].geomFilePath
        // markerMap[9876].geomFilePath,
        // markerMap[9875].geomFilePath
    };

    ftkGeometry geom;
        for ( const auto& geomFile : geomFiles )
    {
        switch ( loadGeometry( lib, sn, geomFile, geom ) )
        {
        case 1:
            cout << "Loaded from installation directory." << endl;

        case 0:
            if ( ftkError::FTK_OK != ftkSetGeometry( lib, sn, &geom ) )
            {
                checkError( lib );
            }
            break;

        default:

            cerr << "Error, cannot load geometry file '"
                << geomFile << "'." << endl;
            if ( ftkError::FTK_OK != ftkClose( &lib ) )
            {
                checkError( lib );
            }

            return 1;
        }
    }

    // ------------------------------------------------------------------------

    // ------------------------------------------------------------------------

    ftkFrameQuery* frame = ftkCreateFrame();  
    if ( frame == 0 )
    {
        error( "Cannot create frame instance"  );
        ROS_WARN( "Cannot create frame instance" );
    }

    ftkError err( ftkSetFrameOptions( false, false, 16u, 16u, 0u, 16u,
                                      frame ) );

    if ( err != ftkError::FTK_OK )
    {
        ftkDeleteFrame( frame );
        checkError( lib);
    }

    uint32 counter( 10u );

    cout.setf( std::ios::fixed, std::ios::floatfield ); 

    while (ros::ok())
    {
        ROS_INFO(" Frame Handler Activated. ");
        /* block up to 100 ms if next frame is not available*/
        err = ftkGetLastFrame( lib, sn, frame, 100u );
        if ( err > ftkError::FTK_OK )
        {
            cout << ".";
            continue;
        }
        else if ( err == ftkError::FTK_WAR_TEMP_INVALID )
        {
            cout << "temperature warning" << endl;
        }
        else if ( err < ftkError::FTK_OK )
        {
            cout << "warning: " << int32( err ) << endl;
            if ( err == ftkError::FTK_WAR_NO_FRAME )
            {
                continue;
            }
        }

        switch ( frame->markersStat )
        {
        case ftkQueryStatus::QS_WAR_SKIPPED:
            ftkDeleteFrame( frame );
            cerr << "marker fields in the frame are not set correctly" << endl;
            checkError( lib);

        case ftkQueryStatus::QS_ERR_INVALID_RESERVED_SIZE:
            ftkDeleteFrame( frame );
            cerr << "frame -> markersVersionSize is invalid" << endl;
            checkError( lib);

        default:
            ftkDeleteFrame( frame );
            cerr << "invalid status" << endl;
            checkError( lib );

        case ftkQueryStatus::QS_OK:
            break;
        }

        if ( frame->markersCount == 0u )
        {
            cout << ".";
            sleep( 1000L );
            continue;
        }

        if ( frame->markersStat == ftkQueryStatus::QS_ERR_OVERFLOW )
        {
            cerr <<
                "WARNING: marker overflow. Please increase cstMarkersCount"
                 << endl;
        }

        for ( int i = 0u; i < frame->markersCount; ++i )
        {
            // cout.precision( 2u );
            int gid = frame->markers[ i ].geometryId;
            // cout << "Detected Marker: " << markerMap[gid].body << endl;

            publishPoses(&frame->markers[ i ]
                        , markerMap[gid].br
                        , markerMap[gid].body
                        , &markerMap[gid].pub
                        , markerMap[gid].parent_frame
                        , markerMap[gid].frame
                        );
        }
        loop_rate.sleep();

    }

    if ( counter != 0u )
    {
        cout << endl << "Acquisition loop aborted after too many vain trials"
             << endl;
    }
    else
    {
        cout << "\tSuccess" << endl;
    }

    // ------------------------------------------------------------------------
    // Close driver

    // ftkDeleteFrame( frame );

    if ( ftkError::FTK_OK != ftkClose( &lib ) )
    {
        checkError( lib );
    }

  return 0;

}