#include "imls_icp.h"
#include <csm/csm_all.h>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "champion_nav_msgs/ChampionNavLaserScan.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include "sensor_msgs/LaserScan.h"

//pcl::visualization::CloudViewer g_cloudViewer("cloud_viewer");
//此处bag包的地址需要自行修改
const std::string bagfile = "/home/kevin/Documents/lidar_SLAM_learning/ch4/PA4/imlsMatcherProject/src/bag/imls_icp.bag";

class CSMDebug
{
public:
    CSMDebug();

private:
    // callback to process laser scan messages
    void championLaserScanCallback(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg);

    // Convert the laser message to LDP type
    void ChampoinLaserScanToLDP(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg, LDP& ldp);

    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

    //发布路径消息
    void pubPath(Eigen::Vector3d& pose, nav_msgs::Path &path, ros::Publisher &mcu_path_pub_);

    void pubPath(const nav_msgs::OdometryConstPtr& msg, nav_msgs::Path &path, ros::Publisher &mcu_path_pub_);

    void SetPLICPParams();
    void LaserScanToLDP(sensor_msgs::LaserScan *pScan,
                                 LDP& ldp);
    Eigen::Vector3d  PLICPBetweenTwoFrames(LDP& currentLDPScan,
                                           Eigen::Vector3d tmprPose);

private:
    bool m_isFirstFrame;
    ros::NodeHandle m_nh;

    Eigen::Vector3d m_prevLaserPose;
    std::vector<Eigen::Vector2d> m_prevPointCloud;
    nav_msgs::Path m_csmPath;
    nav_msgs::Path m_odomPath;

    tf::TransformListener m_tfListener;
    ros::NodeHandle m_node;

    ros::Subscriber m_laserscanSub;
    ros::Publisher m_csmPathPub;
    ros::Publisher m_odomPathPub;

    LDP m_prevLDP;
    sm_params m_PLICPParams;
    sm_result m_OutputResult;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "csm_debug");

    CSMDebug imls_debug;

    ros::spin();

    return (0);
}

CSMDebug::CSMDebug()
{
    m_csmPathPub = m_node.advertise<nav_msgs::Path>("csm_path_pub_",1,true);
    m_csmPath.header.stamp = ros::Time::now();
    m_csmPath.header.frame_id = "odom";
    m_odomPathPub = m_node.advertise<nav_msgs::Path>("odom_path_pub_",1,true);
    m_odomPath.header.stamp = ros::Time::now();
    m_odomPath.header.frame_id = "odom";

    m_isFirstFrame = true;

    SetPLICPParams();

    rosbag::Bag bag;
    bag.open(bagfile, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/sick_scan"));
    topics.push_back(std::string("/odom"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    //按顺序读取bag内激光的消息和里程计的消息
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        champion_nav_msgs::ChampionNavLaserScanConstPtr scan = m.instantiate<champion_nav_msgs::ChampionNavLaserScan>();
        if(scan != NULL)
            championLaserScanCallback(scan);

        nav_msgs::OdometryConstPtr odom = m.instantiate<nav_msgs::Odometry>();
        if(odom != NULL)
            odomCallback(odom);

        ros::spinOnce();
        if(!ros::ok())
            break;
    }
    // m_laserscanSub = m_nh.subscribe("sick_scan",5,&imlsDebug::championLaserScanCallback,this);
}

// callback to process laser scan messages
void CSMDebug::championLaserScanCallback(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg)
{
    static Eigen::Vector3d initialGuess(0, 0, 0);

    if(m_isFirstFrame == true)
    {
        m_isFirstFrame = false;
        m_prevLaserPose = Eigen::Vector3d(0, 0, 0);
        pubPath(m_prevLaserPose, m_csmPath, m_csmPathPub);
        ChampoinLaserScanToLDP(msg, m_prevLDP);
        return;
    }

    LDP currentLDP;
    ChampoinLaserScanToLDP(msg, currentLDP);
        
    Eigen::Vector3d relativeLaserPose = PLICPBetweenTwoFrames(currentLDP, initialGuess);
    initialGuess = relativeLaserPose;

    Eigen::Matrix3d lastPose;
    lastPose << cos(m_prevLaserPose(2)), -sin(m_prevLaserPose(2)), m_prevLaserPose(0),
        sin(m_prevLaserPose(2)), cos(m_prevLaserPose(2)), m_prevLaserPose(1),
        0, 0, 1;

    Eigen::Matrix3d rPose;
    rPose << cos(relativeLaserPose(2)), -sin(relativeLaserPose(2)), relativeLaserPose(0),
        sin(relativeLaserPose(2)), cos(relativeLaserPose(2)), relativeLaserPose(1),
        0, 0, 1;

    Eigen::Matrix3d nowPose = lastPose * rPose;
    m_prevLaserPose << nowPose(0, 2), nowPose(1, 2), atan2(nowPose(1, 0), nowPose(0, 0));
    pubPath(m_prevLaserPose, m_csmPath, m_csmPathPub);
}

// Convert the laser message to LDP type
void CSMDebug::ChampoinLaserScanToLDP(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg, LDP& ldp) 
{
    ldp = ld_alloc_new(msg->ranges.size());

    // Store range and angle data
    for (int i = 0; i < msg->ranges.size(); ++i) {
        double dist = msg->ranges[i];

        if (msg->ranges[i] > msg->range_min and msg->ranges[i] < msg->range_max) {
            ldp->valid[i] = 1;
            ldp->readings[i] = dist;
        }
        else {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1;
        }

        ldp->theta[i] = msg->angles[i];
    }

    // Store min and max angles
    ldp->min_theta = msg->angle_min;
    ldp->max_theta = msg->angle_max;
        
    // Set odometry data to 0
    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}

void CSMDebug::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    if(m_isFirstFrame == true)
        return;

    pubPath(msg, m_odomPath, m_odomPathPub);
}

//发布路径消息
void CSMDebug::pubPath(Eigen::Vector3d& pose, nav_msgs::Path &path, ros::Publisher &mcu_path_pub_)
{
    ros::Time current_time = ros::Time::now();
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = pose(0);
    this_pose_stamped.pose.position.y = pose(1);

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose(2));
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp = current_time;
    this_pose_stamped.header.frame_id = "odom";
    path.poses.push_back(this_pose_stamped);
    mcu_path_pub_.publish(path);
}

void CSMDebug::pubPath(const nav_msgs::OdometryConstPtr& msg, nav_msgs::Path &path, ros::Publisher &mcu_path_pub_)
{
    ros::Time current_time = ros::Time::now();
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = msg->pose.pose.position.x;
    this_pose_stamped.pose.position.y = msg->pose.pose.position.y;

    this_pose_stamped.pose.orientation.x = msg->pose.pose.orientation.x;
    this_pose_stamped.pose.orientation.y = msg->pose.pose.orientation.y;
    this_pose_stamped.pose.orientation.z = msg->pose.pose.orientation.z;
    this_pose_stamped.pose.orientation.w = msg->pose.pose.orientation.w;

    this_pose_stamped.header.stamp = current_time;
    this_pose_stamped.header.frame_id = "odom";
    path.poses.push_back(this_pose_stamped);
    mcu_path_pub_.publish(path);
}


//设置PL-ICP的参数
void CSMDebug::SetPLICPParams()
{
    //设置激光的范围
    m_PLICPParams.min_reading = 0.1;
    m_PLICPParams.max_reading = 20;

    //设置位姿最大的变化范围
    m_PLICPParams.max_angular_correction_deg = 20.0;
    m_PLICPParams.max_linear_correction = 1;

    //设置迭代停止的条件
    m_PLICPParams.max_iterations = 50;
    m_PLICPParams.epsilon_xy = 0.000001;
    m_PLICPParams.epsilon_theta = 0.0000001;

    //设置correspondence相关参数
    m_PLICPParams.max_correspondence_dist = 1;
    m_PLICPParams.sigma = 0.01;
    m_PLICPParams.use_corr_tricks = 1;

    //设置restart过程，因为不需要restart所以可以不管
    m_PLICPParams.restart = 0;
    m_PLICPParams.restart_threshold_mean_error = 0.01;
    m_PLICPParams.restart_dt = 1.0;
    m_PLICPParams.restart_dtheta = 0.1;

    //设置聚类参数
    m_PLICPParams.clustering_threshold = 0.2;

    //用最近的10个点来估计方向
    m_PLICPParams.orientation_neighbourhood = 10;

    //设置使用PL-ICP
    m_PLICPParams.use_point_to_line_distance = 1;

    //不进行alpha_test
    m_PLICPParams.do_alpha_test = 0;
    m_PLICPParams.do_alpha_test_thresholdDeg = 5;

    //设置trimmed参数 用来进行outlier remove
    m_PLICPParams.outliers_maxPerc = 0.9;
    m_PLICPParams.outliers_adaptive_order = 0.7;
    m_PLICPParams.outliers_adaptive_mult = 2.0;

    //进行visibility_test 和 remove double
    m_PLICPParams.do_visibility_test = 1;
    m_PLICPParams.outliers_remove_doubles = 1;
    m_PLICPParams.do_compute_covariance = 0;
    m_PLICPParams.debug_verify_tricks = 0;
    m_PLICPParams.use_ml_weights = 0;
    m_PLICPParams.use_sigma_weights = 0;
}

//把激光雷达数据 转换为PL-ICP需要的数据
void CSMDebug::LaserScanToLDP(sensor_msgs::LaserScan *pScan,
                           LDP& ldp)
{
    int nPts = pScan->intensities.size();
    ldp = ld_alloc_new(nPts);

    for(int i = 0;i < nPts;i++)
    {
        double dist = pScan->ranges[i];
        if(dist > 0.1 && dist < 20)
        {
            ldp->valid[i] = 1;
            ldp->readings[i] = dist;
        }
        else
        {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1;
        }
        ldp->theta[i] = pScan->angle_min+pScan->angle_increment*i;
    }
    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[nPts-1];

    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}


//求两帧之间的icp位姿匹配
Eigen::Vector3d CSMDebug::PLICPBetweenTwoFrames(LDP& currentLDPScan,
                                              Eigen::Vector3d tmprPose)
{
    m_prevLDP->odometry[0] = 0.0;
    m_prevLDP->odometry[1] = 0.0;
    m_prevLDP->odometry[2] = 0.0;

    m_prevLDP->estimate[0] = 0.0;
    m_prevLDP->estimate[1] = 0.0;
    m_prevLDP->estimate[2] = 0.0;

    m_prevLDP->true_pose[0] = 0.0;
    m_prevLDP->true_pose[1] = 0.0;
    m_prevLDP->true_pose[2] = 0.0;

    //设置匹配的参数值
    m_PLICPParams.laser_ref = m_prevLDP;
    m_PLICPParams.laser_sens = currentLDPScan;

    m_PLICPParams.first_guess[0] = tmprPose(0);
    m_PLICPParams.first_guess[1] = tmprPose(1);
    m_PLICPParams.first_guess[2] = tmprPose(2);

    m_OutputResult.cov_x_m = 0;
    m_OutputResult.dx_dy1_m = 0;
    m_OutputResult.dx_dy2_m = 0;

    sm_icp(&m_PLICPParams,&m_OutputResult);

    //nowPose在lastPose中的坐标
    Eigen::Vector3d  rPose;
    if(m_OutputResult.valid)
    {
        //得到两帧激光之间的相对位姿
        rPose(0)=(m_OutputResult.x[0]);
        rPose(1)=(m_OutputResult.x[1]);
        rPose(2)=(m_OutputResult.x[2]);

//        std::cout <<"Iter:"<<m_OutputResult.iterations<<std::endl;
//        std::cout <<"Corr:"<<m_OutputResult.nvalid<<std::endl;
//        std::cout <<"Erro:"<<m_OutputResult.error<<std::endl;

//        std::cout <<"PL ICP GOOD"<<std::endl;
    }
    else
    {
        std::cout <<"PL ICP Failed!!!!!!!"<<std::endl;
        rPose = tmprPose;
    }

    //更新

    //ld_free(m_prevLDP);

    m_prevLDP = currentLDPScan;

    return rPose;
}