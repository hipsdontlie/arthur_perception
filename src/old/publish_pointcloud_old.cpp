/*
 * transform_marker_broadcast
 *
 * Hipster (c) 2022-2023
 * author: Gunjan Sethi, gunjans@andrew.cmu.edu
 *         Kaushik Balasundar, kaushik@cmu.edu
 *     
 */

// Include libraries for ROS
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include "std_msgs/Header.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
// #include <sensor_msgs>
#include <eigen3/Eigen/Dense>

#include <tf2_ros/transform_broadcaster.h>

// Include libraries for camera
#include <algorithm>
#include <deque>
#include <iomanip>
#include <iostream>
#include "helpers.hpp"
#include <sstream>


#include "geometryHelper.hpp"
using namespace std;

struct euler_representation{
    float64 r00, r01, r02;
    float64 r10, r11, r12;
    float64 r20, r21, r22;
};

// Camera variables
static const unsigned ENABLE_ONBOARD_PROCESSING_OPTION = 6000;
static const unsigned SENDING_IMAGES_OPTION = 6003;

ftkLibrary lib = 0;
uint64 sn( 0uLL );

// Configuration file
string cfgFile( "" );
// Geometry file
deque< string > geomFiles{ "/home/mrsd-team-c/arthur_ws/src/arthur_perception/data/geometry9990.ini" };

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
int publishData(tf2_ros::TransformBroadcaster br, ros::Publisher* pointcloud_pub, ros::Publisher* marker_pose_publisher, ftkMarker* marker
                , sensor_msgs::PointCloud2 cloud
                , sensor_msgs::PointCloud2Modifier modifier
                , sensor_msgs::PointCloud2Iterator<float> iter_x
                , sensor_msgs::PointCloud2Iterator<float> iter_y
                , sensor_msgs::PointCloud2Iterator<float> iter_z){
    
    
    cout<<"\nBroadcasting";  

    // Declare new TransformStamped message
    geometry_msgs::TransformStamped marker_transform;

    // Initialize transform frame header
    ros::Time stamp = ros::Time::now(); 
    string frame_id = "geometry " + marker->geometryId;
    marker_transform.header.stamp = stamp;
    marker_transform.header.frame_id = "camera";
    marker_transform.child_frame_id = "marker";


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

    marker_transform.transform.translation.x = marker->translationMM[ 0u ]/1000.00;
    marker_transform.transform.translation.y = marker->translationMM[ 1u ]/1000.00;
    marker_transform.transform.translation.z = marker->translationMM[ 2u ]/1000.00;

    marker_transform.transform.rotation.x = q.x();
    marker_transform.transform.rotation.y = q.y();
    marker_transform.transform.rotation.z = q.z();
    marker_transform.transform.rotation.w = q.w();

    // Broadcast
    br.sendTransform(marker_transform);

    *iter_x = marker->translationMM[ 0u ];//1000.00;
    *iter_y = marker->translationMM[ 1u ];//1000.00;
    *iter_z = marker->translationMM[ 2u ];//1000.00;   
    

    ++iter_x;
    ++iter_y;
    ++iter_z;

    pointcloud_pub->publish(cloud);

    return 0;

}

int main(int argc, char **argv)
{
    // ------------------------------------------------------------------------
    // ROS Initialization
     ros::init(argc, argv, "transform_marker_broadcast");
     ros::NodeHandle n;

     ros::Publisher marker_pose_publisher = n.advertise<geometry_msgs::Pose>("marker_poses", 1000);
     ros::Publisher pointcloud_pub = n.advertise<sensor_msgs::PointCloud2>("pc", 1000);
     tf2_ros::TransformBroadcaster marker_broadcaster;
     ros::Rate loop_rate(60);


    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.resize(1);
    modifier.setPointCloud2Fields(
    3, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
    sensor_msgs::PointField::FLOAT32, "z", 1,
    sensor_msgs::PointField::FLOAT32);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    // ------------------------------------------------------------------------
    // Camera Initialization

    cout<<"Running connection test node\n";
  // Initialize camera driver
    lib = initializeCameraDriver(cfgFile); 
    if(lib==nullptr){
        cout << "Failed to initialize driver";
        return 1;
    }

  // discover camera
    DeviceData camera = discoverCamera(lib);
    uint64 sn( camera.SerialNumber );

  // Set current temperature index
    ftkSetInt32( lib, sn, 30, 1 );

    // ------------------------------------------------------------------------
    // When using a spryTrack, onboard processing of the images is preferred.
    // Sending of the images is disabled so that the sample operates on a USB2
    // connection
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
    // Set geometry

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
    // Initialize the frame to get marker pose

    ftkFrameQuery* frame = ftkCreateFrame();

    if ( frame == 0 )
    {
        error( "Cannot create frame instance"  );
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

        ftk3DFiducial* fiducials;
        cout<<"--"<<endl;
        cout<<"Raw Data Count- "<<endl;
        cout<<"Left Count: "<<frame->rawDataLeftCount<<endl;
        cout<<"Right Count: "<<frame->rawDataRightCount<<endl;
        cout<<"Fiducials Count- "<<frame->threeDFiducialsCount<<endl;

        if ( frame->threeDFiducialsStat == ftkQueryStatus::QS_OK)
        {
            cout<<"got fids!";
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
            cout<<"Found!"<<endl;
            cout << endl;
            cout.precision( 2u );
            cout << "geometry " << frame->markers[ i ].geometryId
                
                << "\n, rotation (" << frame->markers[ i ].rotation[ 0u ][ 0u ]
                << " " << frame->markers[ i ].rotation[ 0u ][ 1u ]
                << " " << frame->markers[ i ].rotation[ 0u ][ 2u ]
                << " " << frame->markers[ i ].rotation[ 1u ][ 0u ]
                << " " << frame->markers[ i ].rotation[ 1u ][ 1u ]
                << " " << frame->markers[ i ].rotation[ 1u ][ 2u ]
                << " " << frame->markers[ i ].rotation[ 2u ][ 0u ]
                << " " << frame->markers[ i ].rotation[ 2u ][ 1u ]
                << " " << frame->markers[ i ].rotation[ 2u ][ 2u ]
                << ")\n"
                
                 << ", trans (" << frame->markers[ i ].translationMM[ 0u ]
                 << " " << frame->markers[ i ].translationMM[ 1u ]
                 << " " << frame->markers[ i ].translationMM[ 2u ]
                 << "), error ";
            cout.precision( 3u );
            cout << frame->markers[ i ].registrationErrorMM << endl;
            publishData(marker_broadcaster, &pointcloud_pub, &marker_pose_publisher, &frame->markers[ i ]
            , cloud
            , modifier
            , iter_x
            , iter_y
            , iter_z);
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
        cout << "\tSUCCESS" << endl;
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