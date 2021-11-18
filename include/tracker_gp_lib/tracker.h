#include <iostream>
#include <string.h>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <cmath>
#include <ctime>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>
#include <sstream>
#include <iomanip>
#include <cstdlib> 

// ros msgs
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/ColorRGBA.h>

// obstacle ros msgs
#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistWithCovariance.h>

// pcl 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/common/geometry.h>

// IHGP
#include "ihgp/InfiniteHorizonGP.hpp"
#include "ihgp/Matern32model.hpp"
 
// visualization
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <limits>
#include <utility>

//*****************************************************************
using namespace std;
using namespace Eigen;

class ObstacleTrack
{
public:

    ObstacleTrack();

    ~ObstacleTrack();

    bool initialize();

    // ROS Publihser
    ros::Publisher obstacle_pub; // obstacle pos&vel
    ros::Publisher marker_pub; // obstacle pose visualization 
    ros::Publisher pose_pub;

private:   
    std::string frame_id_;
    bool firstFrame = true;
    std::vector<int> objIDs; // obj lists
    std::vector<std::vector<pcl::PointXYZI>> stack_obj; // t~(t-10) cluster Centers stack
    std::vector<std_msgs::ColorRGBA> colorset; // rviz msg colorset

    tf::TransformListener tf_listener_;
    std::vector<tf::StampedTransform> ego_tf_stack;

    // IHGP state space model
    int next_obj_num = 0;
    int spin_counter = 0;
    float dt_gp = 1/frequency; 
    std::vector<InfiniteHorizonGP*> GPs_x;
    std::vector<InfiniteHorizonGP*> GPs_y;
    
    // Configureation (ROS parameter)
    float frequency = 10; // node frequency
    float id_thershold;
    int data_length=10;

    float lpf_tau; 
    double logSigma2_x;
    double logMagnSigma2_x;
    double logLengthScale_x;

    double logSigma2_y;
    double logMagnSigma2_y;
    double logLengthScale_y;

    ros::NodeHandle nh_;    

    void updateParam();

    void track(const sensor_msgs::PointCloud2ConstPtr& input);

    void publishObstacles(std::vector<std::vector<pcl::PointXYZI>> pos_vel_s, \
                        std::string frame_id, \
                        std::vector<int> this_objIDs);

    void publishMarkers(std::vector<std::vector<pcl::PointXYZI>> pos_vel_s, \
                        std::string frame_id, \
                        std::vector<int> this_objIDs);

    void registerNewObstacle(pcl::PointXYZI centroid);

    void unregisterOldObstacle(double now);

    void updateObstacleQueue(const int i, pcl::PointXYZI centroid);

    void fill_with_linear_interpolation(const int i, pcl::PointXYZI centroid);

    float quaternion2eularYaw(geometry_msgs::Quaternion map_angle);

    float euc_dist(Vector3d P1, Vector3d P2);

    std::vector<std::vector<pcl::PointXYZI>> callIHGP(std::vector<int> this_objIDs);

    pcl::PointXYZI LPF_pos(std::vector<pcl::PointXYZI> centroids);

    pcl::PointXYZI IHGP_fixed_pos(std::vector<pcl::PointXYZI> centroids, int n);

    pcl::PointXYZI IHGP_fixed_vel(std::vector<pcl::PointXYZI> centroids, int n);

};
