#ifndef init_core_navigation_H
#define init_core_navigation_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <geometry_utils/Transform3.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/time_synchronizer.h>


class InitCoreNav {
public:

        typedef sensor_msgs::Imu ImuData;
        typedef geometry_msgs::PoseStamped PoseData;

        typedef Eigen::VectorXd Vector;
        typedef Eigen::MatrixXd Matrix;
        typedef Eigen::Matrix<double, 3, 1> Vector3;
        typedef Eigen::Matrix<double, 6, 1> Vector6;
        InitCoreNav::Vector6 imu;

        InitCoreNav();
        ~InitCoreNav();

// Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
        bool Initialize(const ros::NodeHandle& n);

private:
// Node initialization
        bool Init(const ros::NodeHandle& n);
        bool LoadParameters(const ros::NodeHandle& n);
        bool RegisterCallbacks(const ros::NodeHandle& n);

// Publish estimated  states.
        void PublishStates(const InitCoreNav::Vector3& states, const ros::Publisher& pub);

        void ImuCallback(const ImuData& imu_data_);
        void Propagate(const InitCoreNav::Vector6& imu);

        InitCoreNav::Vector6 getImuData(const ImuData& imu_data_);

// The node's name.
        std::string name_;

// Subscriber
        ros::Subscriber imu_sub_;


// Publisher.
        ros::Publisher bias_a_pub_;
        ros::Publisher bias_g_pub_;
        tf::TransformBroadcaster transformed_states_tf_broad;

        ImuData imu_data_;
        bool has_imu_ = false;
        bool first_imu_ = true;

// Most recent time stamp for publishers.
        ros::Time stamp_;

// Coordinate frames.
        std::string frame_id_out_;
        std::string frame_id_imu_;
        std::string frame_id_fixed_;

// update rate [hz]
        unsigned int publish_hz_;
        unsigned int sensor_pub_rate_;

// sub. topics
        std::string imu_topic_;

// For initialization.
        bool initialized_;

// Filter vars.
        int num_states_ = 15;
        InitCoreNav::Vector3 bias_a_;
        InitCoreNav::Vector3 bias_g_;
        InitCoreNav::Vector3 ins_bias_a;
        InitCoreNav::Vector3 ins_bias_g;

        InitCoreNav::Vector3 omega_b_ib_;
        InitCoreNav::Vector3 f_ib_b_;

// filter noise params
        double position_noise_, attitude_noise_, velocity_noise_, bias_noise_;

// initial pose

        double init_ba_x, init_ba_y, init_ba_z, init_bg_x, init_bg_y, init_bg_z;


        double imu_stamp_curr_, imu_stamp_prev_;
        double dt_imu_;
        int count=0;

};
#endif
