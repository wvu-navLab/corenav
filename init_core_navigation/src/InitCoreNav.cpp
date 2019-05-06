#include <init_core_navigation/InitCoreNav.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <geometry_msgs/TransformStamped.h>

namespace pu = parameter_utils;
namespace gu = geometry_utils;
namespace gr = gu::ros;

InitCoreNav::InitCoreNav() : initialized_(false){
}

InitCoreNav::~InitCoreNav(){
}

bool InitCoreNav::Initialize(const ros::NodeHandle& n){
        name_ = ros::names::append(n.getNamespace(), "InitCoreNav");

        if(!LoadParameters(n)) {
                ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
                return false;
        }

        if(!Init(n)) {
                ROS_ERROR("%s: Failed to initialize.", name_.c_str());
                return false;
        }

        if(!RegisterCallbacks(n)) {
                ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
                return false;
        }

        return true;
}

bool InitCoreNav::Init(const ros::NodeHandle& n){

        bias_a_(0) = init_ba_x;   bias_a_(1) = init_ba_y; bias_a_(2) = init_ba_z;
        bias_g_(0) = init_bg_x;   bias_g_(1) = init_bg_y; bias_g_(2) = init_bg_z;

        return true;
}

bool InitCoreNav::LoadParameters(const ros::NodeHandle& n){
        // Load frame ids.
        if(!pu::Get("frames/frame_id_out", frame_id_out_)) return false;
        if(!pu::Get("frames/frame_id_imu", frame_id_imu_)) return false;
        if(!pu::Get("frames/frame_id_fixed", frame_id_fixed_)) return false;

        // Load topics
        if(!pu::Get("imu/topic", imu_topic_)) return false;

        // Load update rate
        if(!pu::Get("imu/publish_hz", publish_hz_)) return false;
        if(!pu::Get("imu/sensor_pub_rate", sensor_pub_rate_)) return false;

        if (!pu::Get("init/bias/accel/x", init_ba_x)) return false;
        if (!pu::Get("init/bias/accel/y", init_ba_y)) return false;
        if (!pu::Get("init/bias/accel/z", init_ba_z)) return false;
        if (!pu::Get("init/bias/gyro/x", init_bg_x)) return false;
        if (!pu::Get("init/bias/gyro/y", init_bg_y)) return false;
        if (!pu::Get("init/bias/gyro/z", init_bg_z)) return false;

        return true;
}


void InitCoreNav::ImuCallback(const ImuData& imu_dataAdis_){
        imu = getImuData(imu_dataAdis_);
        has_imu_ = true;
        if (first_imu_)
        {
                imu_stamp_prev_ = (imu_dataAdis_.header.stamp).toSec();
                first_imu_ = false;
        }
        else{
                imu_stamp_curr_ = (imu_dataAdis_.header.stamp).toSec();
                if (count<=200) {
                  Propagate(imu);
                  imu_stamp_prev_  = imu_stamp_curr_;
                } else {

                  if (ros::ok()) {
                    ros::shutdown();
                  }

                }

        }
        return;
}

bool InitCoreNav::RegisterCallbacks(const ros::NodeHandle& n){
        // Create a local nodehandle to manage callback subscriptions.
        ros::NodeHandle nl(n);

        bias_a_pub_ = nl.advertise<geometry_msgs::PointStamped>( "bias_a", 10, false);
        bias_g_pub_ = nl.advertise<geometry_msgs::PointStamped>( "bias_g", 10, false);

        imu_sub_ = nl.subscribe(imu_topic_,  10, &InitCoreNav::ImuCallback, this);

        return true;
}

void InitCoreNav::Propagate(const InitCoreNav::Vector6& imu){ // no joint
        dt_imu_ = imu_stamp_curr_ - imu_stamp_prev_;
        count++;

          omega_b_ib_(0) += imu[3];
          omega_b_ib_(1) += imu[4];
          omega_b_ib_(2) += imu[5];
          f_ib_b_(0) += imu[0];
          f_ib_b_(1) += imu[1];
          f_ib_b_(2) += imu[2];

          ins_bias_a(0)= f_ib_b_(0)/count;
          ins_bias_a(1)= f_ib_b_(1)/count;
          ins_bias_a(2)= f_ib_b_(2)/count;

          ins_bias_g(0)= omega_b_ib_(0)/count;
          ins_bias_g(1)= omega_b_ib_(1)/count;
          ins_bias_g(2)= omega_b_ib_(2)/count;

if (count==200) {
  PublishStates(ins_bias_a, bias_a_pub_);
  PublishStates(ins_bias_g, bias_g_pub_);
}
// else {
//   std::cout << "%";
//   std::cout <<(count/200)*100 << '\n';
// }

        return;
}

InitCoreNav::Vector6 InitCoreNav::getImuData(const ImuData& imu_dataAdis_)
{
        InitCoreNav::Vector6 imuVec((Vector(6) << imu_dataAdis_.linear_acceleration.y*(-1.0),
                                 imu_dataAdis_.linear_acceleration.x,
                                 imu_dataAdis_.linear_acceleration.z,
                                 imu_dataAdis_.angular_velocity.y*(-1.0),
                                 imu_dataAdis_.angular_velocity.x,
                                 imu_dataAdis_.angular_velocity.z).finished());
        return imuVec;
}

// Publish estimated states in global frame
void InitCoreNav::PublishStates(const InitCoreNav::Vector3& states,
                            const ros::Publisher& pub){
        // // Check for subscribers before doing any work.
        if(pub.getNumSubscribers() == 0)
                return;

        geometry_msgs::PointStamped msg;

        msg.point.x = states(0);
        msg.point.y = states(1);
        msg.point.z = states(2);
        msg.header.frame_id = frame_id_imu_;
        msg.header.stamp = ros::Time::now();

        pub.publish(msg);
}
