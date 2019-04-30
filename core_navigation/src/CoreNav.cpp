/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: Cagri, Ryan
 */

#include <core_navigation/CoreNav.h>
#include <core_navigation/InsConst.h>
#include <core_navigation/InsUtils.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <geometry_msgs/TransformStamped.h>

namespace pu = parameter_utils;
namespace gu = geometry_utils;
namespace gr = gu::ros;

CoreNav::CoreNav() : initialized_(false){
}

CoreNav::~CoreNav(){
}

bool CoreNav::Initialize(const ros::NodeHandle& n){
        name_ = ros::names::append(n.getNamespace(), "CoreNav");

        if(!LoadParameters(n)) {
                ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
                return false;
        }

        if(!Init(n)) {
                ROS_ERROR("%s: Failed to initialize GTSAM.", name_.c_str());
                return false;
        }

        if(!RegisterCallbacks(n)) {
                ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
                return false;
        }

        return true;
}

bool CoreNav::Init(const ros::NodeHandle& n){

        error_states_ = Eigen::VectorXd::Zero(num_states_);

        // // Construct initial P matrix, Diagonal P
        P_= Eigen::MatrixXd::Zero(num_states_,num_states_);

        // TODO:: Don't hardcode these params! Changes for every test
        Eigen::MatrixXd P= Eigen::MatrixXd::Zero(15,15);
        P(0,0)=1.218469679146834e-06; P(1,1)=1.218469679146834e-06; P(2,2)=4.873878716587337e-06;
        P(3,3)=4.000000000000001e-07; P(4,4)=4.000000000000001e-07; P(5,5)=4.000000000000001e-07;
        P(6,6)=1.21846967914683e-14; P(7,7)=1.21846967914683e-14; P(8,8)=100.0;
        P(9,9)=-0.011428207234497;   P(10,10)=0.008740805264598; P(11,11)=0.013108032973440;
        P(12,12)=-8.805993629556402e-04; P(13,13)=9.874592038800420e-04;  P(14,14)=8.985670378275259e-04;

        ba_(0) = P(9,9);
        ba_(1) = P(10,10);
        ba_(2) = P(11,11);
        bg_(0) = P(12,12);
        bg_(1) = P(13,13);
        bg_(2) = P(14,14);

        R_ << 0.0004,0,0,0,
        0,0.1152,0,0,
        0,0,0.0025,0,
        0,0,0,0.0025;

        R_zupt << std::pow(0.012,2),0,0, // TODO: Revisit here
        0,std::pow(0.012,2),0,
        0,0,std::pow(0.012,2);

        R_zaru << std::pow(0.01,2),0,0, // TODO: Revisit here
        0,std::pow(0.01,2),0,
        0,0,std::pow(0.0025,2);
        R_holo << 0.05,0,0,0.05;

        H11_<< 0.0,0.0,0.0;
        H12_ << 0.0,0.0,0.0;
        H21_<< 0.0,0.0,0.0;
        H31_<< 0.0,0.0,0.0;
        H32_<< 0.0,0.0,0.0;
        H24_ << 0.0,0.0,0.0;
        H41_ << 0.0,0.0,0.0;
        H42_ << 0.0,0.0,0.0;

        H_zupt << 0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0;

        H_zaru << 0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1;

        ins_att_ << init_roll, init_pitch, init_yaw;
        ins_vel_ << init_vx, init_vy, init_vz;
        ins_pos_ << init_x, init_y, init_z;
        //ins_enu_ <<0.0,0.0,0.0;
        return true;
}

bool CoreNav::LoadParameters(const ros::NodeHandle& n){
        // Load frame ids.
        if(!pu::Get("frames/frame_id_out", frame_id_out_)) return false;
        if(!pu::Get("frames/frame_id_imu", frame_id_imu_)) return false;
        if(!pu::Get("frames/frame_id_odo", frame_id_odo_)) return false;
        if(!pu::Get("frames/frame_id_fixed", frame_id_fixed_)) return false;

        // Load topics
        if(!pu::Get("imu/topic", imu_topic_)) return false;
        if(!pu::Get("odo/topic", odo_topic_)) return false;
        if(!pu::Get("joint/topic", joint_topic_)) return false;

        // Load update rate
        if(!pu::Get("imu/publish_hz", publish_hz_)) return false;
        if(!pu::Get("imu/sensor_pub_rate", sensor_pub_rate_)) return false;

        // Load imu noise specs
        if(!pu::Get("imu/noise/accel_noise_sigma", accel_sigma_)) return false;
        if(!pu::Get("imu/noise/gyro_noise_sigma", gyro_sigma_)) return false;
        if(!pu::Get("imu/noise/accel_bias_rw_sigma", accel_rw_)) return false;
        if(!pu::Get("imu/noise/gyro_bias_rw_sigma", gyro_rw_)) return false;

        // Load initial position and orientation.
        if (!pu::Get("init/position/x", init_x)) return false;
        if (!pu::Get("init/position/y", init_y)) return false;
        if (!pu::Get("init/position/z", init_z)) return false;
        if (!pu::Get("init/velocity/x", init_vx)) return false;
        if (!pu::Get("init/velocity/y", init_vy)) return false;
        if (!pu::Get("init/velocity/z", init_vz)) return false;
        if (!pu::Get("init/orientation/x", init_roll)) return false;
        if (!pu::Get("init/orientation/y", init_pitch)) return false;
        if (!pu::Get("init/orientation/z", init_yaw)) return false;

        if (!pu::Get("wheel/T_r_", T_r_)) return false;
        if (!pu::Get("wheel/s_or_", s_or_)) return false;
        if (!pu::Get("wheel/s_delta_or_", s_delta_or_)) return false;

        return true;
}

void CoreNav::OdoCallback(const OdoData& odo_data_){
        odo = getOdoData(odo_data_);
        has_odo_ = true;
        if (first_odo_)
        {
                odo_stamp_prev_ = (odo_data_.header.stamp).toSec();
                first_odo_ = false;
        }
        else{
                odo_stamp_curr_ = (odo_data_.header.stamp).toSec();

                // Update(odo);
                odo_stamp_prev_ = odo_stamp_curr_;
        }
        return;
}

void CoreNav::JointCallBack(const JointData& joint_data_){
        joint = getJointData(joint_data_);
        has_joint_ = true;
        if (first_joint_)
        {
                joint_stamp_prev_ = (joint_data_.header.stamp).toSec();
                first_joint_ = false;
        }
        joint_stamp_curr_ = (joint_data_.header.stamp).toSec();
        return;
}

void CoreNav::ImuCallback(const ImuData& imu_dataAdis_){
        imu = getImuData(imu_dataAdis_);
        has_imu_ = true;
        if (first_imu_)
        {
                imu_stamp_prev_ = (imu_dataAdis_.header.stamp).toSec();
                first_imu_ = false;
        }
        else{
                imu_stamp_curr_ = (imu_dataAdis_.header.stamp).toSec();
                Propagate(imu,joint);
                imu_stamp_prev_  = imu_stamp_curr_;
        }
        return;
}

bool CoreNav::RegisterCallbacks(const ros::NodeHandle& n){
        // Create a local nodehandle to manage callback subscriptions.
        ros::NodeHandle nl(n);

        position_pub_ = nl.advertise<geometry_msgs::PointStamped>( "position", 25, false);
        velocity_pub_ = nl.advertise<geometry_msgs::PointStamped>( "velocity", 25, false);
        attitude_pub_ = nl.advertise<geometry_msgs::PointStamped>( "attitude", 25, false);
        enu_pub_ = nl.advertise<geometry_msgs::PointStamped>( "enu", 25, false);

        imu_sub_ = nl.subscribe(imu_topic_,  10, &CoreNav::ImuCallback, this);
        odo_sub_ = nl.subscribe(odo_topic_,  10, &CoreNav::OdoCallback, this);
        joint_sub_ = nl.subscribe(joint_topic_,  10, &CoreNav::JointCallBack, this);

        return true;
}

void CoreNav::Propagate(const CoreNav::Vector6& imu, const CoreNav::Vector4& joint){
        rearVel_ = (joint[3]+joint[2])*INS::wheel_radius/2.0;
        headRate_ = ((joint[2]+joint[0])/2.0-(joint[3]+joint[1])/2.0)*INS::wheel_radius/(0.545);
        dt_imu_ = imu_stamp_curr_ - imu_stamp_prev_;
        count++;
        omega_b_ib_ << imu[3] - bg_(0), imu[4]-bg_(1), imu[5]- bg_(2);
        f_ib_b_ << imu[0]- ba_(0), imu[1] - ba_(1), imu[2] - ba_(2);
//------------------------------------------------------------------------------
        //Attitude Update---------------------------------------------------------------
        // input =insAtt(:,i-1),omega_ie,insLLH(:,i-1),omega_b_ib,ecc,Ro,insVel(:,i-1),dtIMU
        // output= insAtt(:,i), Cb2n+,Cb2n-,Omega_n_en,Omega_n_ie,R_N,R_E,omega_n_in,omega_n_ie
        //------------------------------------------------------------------------------
        CoreNav::Matrix3 CnbMinus = CoreNav::eul_to_dcm(ins_att_[0],ins_att_[1],ins_att_[2]);
        CbnMinus=CnbMinus.transpose();
        omega_n_ie_ << INS::omega_ie*cos(ins_pos_[0]), 0.0, (-1.0)*INS::omega_ie*sin(ins_pos_[0]); // Checked
        Omega_b_ib_ = CoreNav::skew_symm( omega_b_ib_ );
        Omega_n_ie_ = CoreNav::skew_symm( omega_n_ie_ );
        // Radius of Curvature for North-South Motion (eq. 2.105)
        double R_N = INS::Ro*(1.0-pow(INS::ecc,2.0))/pow(1.0-pow(INS::ecc,2.0)*pow(sin(ins_pos_(0)),2.0),(3.0/2.0));
        // Radius of Curvature in East-West Direction (eq. 2.106)
        double R_E = INS::Ro/sqrt(1.0-pow(INS::ecc,2.0)*pow(sin(ins_pos_(0)),2.0));
        //rotation rate vector

        omega_n_en_ << ins_vel_(1)/(R_E+ins_pos_(2)),
        (-1.0)*ins_vel_(0)/(R_N+ins_pos_(2)),
        ((-1.0)*ins_vel_(1)*tan(ins_pos_(0)))/(R_E+ins_pos_(2));


        Omega_n_en_ = CoreNav::skew_symm( omega_n_en_ );
        //eq. 2.48
        omega_n_in_ = omega_n_en_ + omega_n_ie_;

        //integrate considering body-rate, Earth-rate, and craft-rate
        CoreNav::Matrix3 eye3=Eigen::Matrix3d::Identity();
        CoreNav::Matrix3 zeros3=Eigen::Matrix3d::Zero(3,3);
        CoreNav::Matrix3 CbnPlus;
        CbnPlus=CbnMinus*(eye3+Omega_b_ib_*dt_imu_)-(Omega_n_ie_+ Omega_n_en_)*CbnMinus*dt_imu_;
        ins_att_= CoreNav::dcm_to_eul( CbnPlus );

        //------------------------------------------------------------------------------
        // Velocity Update -------------------------------------------------------------
        // input= Cb2nMinus, Cb2nPlus, v_ib_b,insVel(:,i-1),insLLH(:,i-1),omega_ie,Ro,ecc,dtIMU
        // output=insVel(:,i)
        //------------------------------------------------------------------------------
        CoreNav::Vector3 V_n_ib;
        //specific-force transformation (eq. 5.48)
        V_n_ib=(1.0/2.0)*(CbnMinus+CbnPlus)*f_ib_b_*dt_imu_;
        grav_ = CoreNav::calc_gravity(ins_pos_(0),ins_pos_(2));
        CoreNav::Vector3 ins_velMinus(ins_vel_);
        ins_vel_= ins_vel_ + V_n_ib +  (grav_ - (Omega_n_en_+2.0*(Omega_n_ie_))*ins_vel_)*dt_imu_;

        //------------------------------------------------------------------------------
        // Position Update -------------------------------------------------------------
        // input= insLLH(:,i-1),insVel(:,i),insVel(:,i-1),Ro,ecc,dtIMU
        // output= insLLH(:,i)
        //------------------------------------------------------------------------------
        double heightPlus=ins_pos_(2)-(dt_imu_/2.0)*(ins_velMinus(2)+ins_vel_(2));
        double latPlus = ins_pos_(0)+(dt_imu_/2.0)*(ins_velMinus(0)/(R_N+ins_pos_(2))+ins_vel_(0)/(R_N+heightPlus));
        double R_EPlus=INS::Ro/sqrt(1.0-pow(INS::ecc,2.0)*pow(sin(latPlus),2.0));
        double lonPlus=ins_pos_(1)+(dt_imu_/2.0)*(ins_velMinus(1)/((R_E+ins_pos_(2))*cos(ins_pos_(0)))+ins_vel_(1)/((R_EPlus+heightPlus)*cos(latPlus)));         //ins_pos_(1)

        ins_pos_ << latPlus,lonPlus,heightPlus;

        //------------------------------------------------------------------------------
        // State Transition Matrix -----------------------------------------------------
        // input= R_EPlus,R_N,insLLH(:,i),insVel(:,i),dtIMU,Cb2nPlus,omega_ie,omega_n_in,f_ib_b
        // output= STM
        //------------------------------------------------------------------------------
        STM_ = CoreNav::insErrorStateModel_LNF(R_EPlus, R_N, ins_pos_, ins_vel_, dt_imu_, CbnPlus, INS::omega_ie,omega_n_in_, f_ib_b_, INS::gravity);

        error_states_ = STM_*error_states_;
        //------------------------------------------------------------------------------
        // Q Matrix --------------------------------------------------------------------
        //------------------------------------------------------------------------------
        Q_ = CoreNav::calc_Q(R_N,R_EPlus,ins_pos_, dt_imu_, CbnPlus, f_ib_b_);
        //------------------------------------------------------------------------------
        // P Matrix --------------------------------------------------------------------
        //------------------------------------------------------------------------------
        P_=STM_*P_*STM_.transpose()+ Q_;
        //------------------------------------------------------------------------------
        // Measurement Matrix ----------------------------------------------------------
        //------------------------------------------------------------------------------
        CoreNav::Matrix3 Cn2bPlus=CbnPlus.transpose();
        CoreNav::Vector3 omega_b_ie=Cn2bPlus*omega_n_ie_;
        CoreNav::Vector3 omega_b_ei=-1.0*omega_b_ie;
        CoreNav::Vector3 omega_b_eb=omega_b_ei+omega_b_ib_;

        CoreNav::Matrix3 ins_vel_ss;         //ins_vel_ skew symmetric
        ins_vel_ss = CoreNav::skew_symm(ins_vel_);
        CoreNav::Vector3 Val_H21(0,cos(ins_att_(0)),sin(ins_att_(0)));

        H11_ += eye3.row(0)*Cn2bPlus*ins_vel_ss*dt_imu_;
        H12_ += eye3.row(0)*Cn2bPlus*dt_imu_;
        H21_ += sin(ins_att_(1))*Val_H21.transpose()*Cn2bPlus*dt_imu_;
        H31_ += eye3.row(1)*Cn2bPlus*ins_vel_ss*dt_imu_;
        H32_ += eye3.row(1)*Cn2bPlus*dt_imu_;
        H41_ += eye3.row(2)*Cn2bPlus*ins_vel_ss*dt_imu_;
        H42_ += eye3.row(2)*Cn2bPlus*dt_imu_;

        CoreNav::Matrix3 Omega_b_eb;
        Omega_b_eb << 0.0, -1.0*omega_b_eb(2), omega_b_eb(1),
        omega_b_eb(2), 0.0, -1.0*omega_b_eb(0),
        -1.0*omega_b_eb(1), omega_b_eb(0), 0.0;

        // Measurement Innovation -- integration part for INS -- eq 16.42
        Vector3 tmp = Cn2bPlus*ins_vel_ + Omega_b_eb*(-0.272*(eye3.col(0)));

        z11_ += tmp[0]*dt_imu_;
        z21_ += cos(ins_att_(1))*dt_imu_;
        z31_ += tmp[1] * dt_imu_;
        z41_ += tmp[2] * dt_imu_;
        CoreNav::NonHolonomic(ins_vel_, ins_att_, ins_pos_, error_states_, P_, omega_b_ib_);

        if ( std::abs(odo[7]) <0.004) { // TODO: Revisit here

                CoreNav::zupt(ins_vel_, ins_att_, ins_pos_, error_states_, P_);
                CoreNav::zaru(ins_vel_, ins_att_, ins_pos_, error_states_, P_, omega_b_ib_);
        }
        if ( headRate_ == 0 ) {
                CoreNav::zaru(ins_vel_, ins_att_, ins_pos_, error_states_, P_, omega_b_ib_);
        }
        ba_(0)=error_states_(9);
        ba_(1)=error_states_(10);
        ba_(2)=error_states_(11);
        bg_(0)=error_states_(12);
        bg_(1)=error_states_(13);
        bg_(2)=error_states_(14);


        double phi = ins_pos_(0);
        double lambda = ins_pos_(1);
        double h = ins_pos_(2);

        double a = 6378137.0000;  //% earth semimajor axis in meters
        double b = 6356752.3142;  //% earth semiminor axis in meters
        double e = sqrt(1-pow((b/a),2));

        double sinphi = sin(phi);
        double cosphi = cos(phi);
        double coslam = cos(lambda);
        double sinlam = sin(lambda);
        double tan2phi = pow((tan(phi)),2);
        double tmp2 = 1 - e*e;
        double tmpden = sqrt( 1 + tmp2*tan2phi );
        double x1 = (a*coslam)/tmpden + h*coslam*cosphi;
        double y1 = (a*sinlam)/tmpden + h*sinlam*cosphi;
        double tmp3 = sqrt(1 - e*e*sinphi*sinphi);
        double z1 = (a*tmp2*sinphi)/tmp3 + h*sinphi;
        Vector3 p1(x1,y1,z1);
        // TODO:: Don't hardcode these params! Changes for every test
        Vector3 p2(856503.7292,-4842984.4396,4047974.3219); // TODO: Revisit here
        Vector3 posDiff = p1 - p2;
        Vector3 orgLLH(init_x, init_y, init_z);
        double sinPhi = sin(orgLLH(0));
        double cosPhi = cos(orgLLH(0));
        double sinLam = sin(orgLLH(1));
        double cosLam = cos(orgLLH(1));
        Matrix R = ( Matrix(3,3) << (-1*sinLam), cosLam, 0, ((-1*sinPhi)*cosLam), ((-1*sinPhi)*sinLam), cosPhi, (cosPhi*cosLam), (cosPhi*sinLam), sinPhi ).finished();
        Vector3 pos;
        pos = R*posDiff;
        ins_enu_ << pos;

        PublishStates(ins_att_, attitude_pub_);
        PublishStates(ins_vel_, velocity_pub_);
        PublishStates(ins_pos_, position_pub_);
        PublishStates(ins_enu_, enu_pub_);
        return;
}

void CoreNav::Update(const CoreNav::Vector13& odo)
{

        dt_odo_ = odo_stamp_curr_ - odo_stamp_prev_;
        double psiOld = init_yaw;
        init_yaw=ins_att_(2);
        CoreNav::Matrix3 Cn2bUnc=CoreNav::eul_to_dcm(ins_att_(0),ins_att_(1),ins_att_(2));
        H11_ =-H11_/dt_odo_;
        H12_ =-H12_/dt_odo_;
        H21_ = H21_*(ins_att_(2)-psiOld)/(dt_odo_*dt_odo_);
        H31_ =-H31_/dt_odo_;
        H32_ =-H32_/dt_odo_;
        H24_ =-(cos(ins_att_(1))*eye3.row(2)*Cn2bUnc.transpose())/dt_odo_;
        H41_ =-H41_/dt_odo_;
        H42_ =-H42_/dt_odo_;

        double z1_odom=rearVel_*(1-s_or_);
        double z2_odom=headRate_*(1-s_or_)-((z11_/dt_odo_)/T_r_)*s_delta_or_;
        double z1_ins=z11_;
        double z2_ins=((ins_att_(2)-psiEst))*z21_; //NOTE psiEst is estimated psi from the odometry update below
        if (abs(z2_ins) > 0.5) {
                z2_ins=0.0;
        }
        double z3_ins=z31_;
        double z4_ins=z41_;
        z11_=z1_odom-z1_ins/dt_odo_;
        z21_=z2_odom-z2_ins/pow(dt_odo_,2.0);
        z31_=0.0-z3_ins/dt_odo_;
        z41_=0.0-z4_ins/dt_odo_;
        Z_<<z11_,z21_,z31_,z41_;

        H_.row(0)<<H11_.transpose(), H12_.transpose(), zeros3.row(0), zeros3.row(0),zeros3.row(0);
        H_.row(1)<<H21_.transpose(), zeros3.row(0), zeros3.row(0), H24_.transpose(), zeros3.row(0);
        H_.row(2)<<H31_.transpose(), H32_.transpose(), zeros3.row(0), zeros3.row(0),zeros3.row(0);
        H_.row(3)<<H41_.transpose(), H42_.transpose(), zeros3.row(0), zeros3.row(0),zeros3.row(0);

        K_ << P_*H_.transpose()*(H_*P_*H_.transpose()+R_).inverse();

        error_states_ = error_states_+K_*(Z_-H_*error_states_);

        CoreNav::Vector3 att_error_states(error_states_(0),error_states_(1),error_states_(2));
        CoreNav::Matrix3 error_states_ss;
        error_states_ss=CoreNav::skew_symm(att_error_states);
        CoreNav::Matrix3 insAttEst;
        insAttEst<<(eye3-error_states_ss)*Cn2bUnc.transpose();

        double phiEst = atan2(insAttEst(2,1),insAttEst(2,2));
        double thetaEst = asin(-insAttEst(2,0));
        double psiEst = atan2(insAttEst(1,0),insAttEst(0,0));

        ins_att_ << phiEst, thetaEst, psiEst;

        ins_vel_ << ins_vel_(0)-error_states_(3), ins_vel_(1)-error_states_(4), ins_vel_(2)-error_states_(5);

        ins_pos_ << ins_pos_(0)-error_states_(6), ins_pos_(1)-error_states_(7), ins_pos_(2)-error_states_(8);

        error_states_(0)=0.0;
        error_states_(1)=0.0;
        error_states_(2)=0.0;

        error_states_(3)=0.0;
        error_states_(4)=0.0;
        error_states_(5)=0.0;

        error_states_(6)=0.0;
        error_states_(7)=0.0;
        error_states_(8)=0.0;
        ba_(0)=error_states_(9);
        ba_(1)=error_states_(10);
        ba_(2)=error_states_(11);
        bg_(0)=error_states_(12);
        bg_(1)=error_states_(13);
        bg_(2)=error_states_(14);


        P_=(Eigen::MatrixXd::Identity(15,15) - K_*H_) * P_ * ( Eigen::MatrixXd::Identity(15,15) - K_ * H_ ).transpose() + K_ * R_ * K_.transpose();

        H11_=zeros3.row(0);
        H12_=zeros3.row(0);
        H21_=zeros3.row(0);
        H31_=zeros3.row(0);
        H32_=zeros3.row(0);
        H24_=zeros3.row(0);
        H41_=zeros3.row(0);
        H42_=zeros3.row(0);

        z11_=0.0;
        z21_=0.0;
        z31_=0.0;
        z41_=0.0;

        return;
}

CoreNav::Vector6 CoreNav::getImuData(const ImuData& imu_dataAdis_)
{
        CoreNav::Vector6 imuVec((Vector(6) << imu_dataAdis_.linear_acceleration.y*(-1.0),
                                 imu_dataAdis_.linear_acceleration.x,
                                 imu_dataAdis_.linear_acceleration.z,
                                 imu_dataAdis_.angular_velocity.y*(-1.0),
                                 imu_dataAdis_.angular_velocity.x,
                                 imu_dataAdis_.angular_velocity.z).finished());

        return imuVec;
}

CoreNav::Vector4 CoreNav::getJointData(const JointData& joint_data_)
{
        CoreNav::Vector4 jointVec((Vector(4) << joint_data_.velocity[0],
                                   joint_data_.velocity[1],
                                   joint_data_.velocity[2],
                                   joint_data_.velocity[3] ).finished());
        return jointVec;
}

CoreNav::Vector13 CoreNav::getOdoData(const OdoData& odo_data_)
{

        CoreNav::Vector13 odoVec( (Vector(13) << odo_data_.pose.pose.position.x,
                                   odo_data_.pose.pose.position.y,
                                   odo_data_.pose.pose.position.z,
                                   odo_data_.pose.pose.orientation.w,
                                   odo_data_.pose.pose.orientation.x,
                                   odo_data_.pose.pose.orientation.y,
                                   odo_data_.pose.pose.orientation.z,
                                   odo_data_.twist.twist.linear.x,
                                   odo_data_.twist.twist.linear.y,
                                   odo_data_.twist.twist.linear.z,
                                   odo_data_.twist.twist.angular.x,
                                   odo_data_.twist.twist.angular.y,
                                   odo_data_.twist.twist.angular.z ).finished() );

        return odoVec;
}


// Publish estimated states in global frame
void CoreNav::PublishStates(const CoreNav::Vector3& states,
                            const ros::Publisher& pub){
        // // Check for subscribers before doing any work.
        if(pub.getNumSubscribers() == 0)
                return;

        geometry_msgs::PointStamped msg;
        msg.point.x = states(0);
        msg.point.y = states(1);
        msg.point.z = states(2);
        msg.header.frame_id = frame_id_fixed_;
        msg.header.stamp = stamp_;
        pub.publish(msg);
}

CoreNav::Vector3 CoreNav::calc_gravity(const double latitude, const double height)
{
        double e2=pow(INS::ecc,2.0);
        double den=1.0-e2*pow(sin(latitude),2.0);
        double Rm=INS::Ro*(1.0-e2)/pow(den,(3.0/2.0));
        double Rp=INS::Ro/pow(den,(1.0/2.0));
        double Top=INS::Ro*pow((pow((1.0-e2),2.0)+pow((sin(latitude)),2.0)+pow((cos(latitude)),2.0)),(1.0/2.0));
        double Bottom=pow((1.0-e2*(pow(sin(latitude),(2.0)))),(1.0/2.0));
        double R_es_e=Top/Bottom;
        double RO=pow(Rp*Rm,(1.0/2.0));
        double g0=9.780318*(1.0+5.3024e-3*pow(sin(latitude),2.0)-5.9e-6*pow(sin(2*latitude),2.0));
        double gravity;
        if(height<0.0)
        {
                gravity=g0*(1.0+height/RO);
        }
        else
        {
                gravity=g0/pow((1.0+height/RO),2.0);
        }
        CoreNav::Vector3 grav(0.0,0.0,gravity);
        return grav;
}

CoreNav::Matrix3 CoreNav::skew_symm(const CoreNav::Vector3 vec)
{
        CoreNav::Matrix3 ss;
        ss << 0.0, -1.0*vec(2), vec(1),
        vec(2), 0.0, -1.0*vec(0),
        -1.0*vec(1), vec(0), 0.0;

        return ss;
}

CoreNav::Matrix3 CoreNav::eul_to_dcm(double phi, double theta, double psi)
{
        double cpsi = cos(psi); double spsi = sin(psi);
        double cthe = cos(theta); double sthe = sin(theta);
        double cphi = cos(phi); double sphi = sin(phi);

        CoreNav::Matrix3 c1; //y
        c1.row(0)<<cpsi, spsi, 0.0; //c11
        c1.row(1)<<(-1.0)*spsi, cpsi,0.0; //c12
        c1.row(2)<<0.0, 0.0,1.0;  //c13

        CoreNav::Matrix3 c2; //p
        c2.row(0)<<cthe,0.0,(-1.0)*sthe; //c21
        c2.row(1)<<0.0,1.0,0.0;   //c22
        c2.row(2)<<sthe,0.0,cthe; //c23

        CoreNav::Matrix3 c3; //r
        c3.row(0)<<1.0,0.0,0.0;   //c31
        c3.row(1)<<0.0,cphi,sphi; //c32
        c3.row(2)<<0.0,(-1.0)*sphi,cphi; //c33

        CoreNav::Matrix3 DCMnb= (c3*c2)*c1; //CnbMinus;nav2body
        return DCMnb;
}

CoreNav::Vector3 CoreNav::dcm_to_eul(CoreNav::Matrix3 dcm)
{
        CoreNav::Vector3 eul;
        eul << std::atan2(dcm(2,1), dcm(2,2)), std::asin(-1*dcm(2,0)), std::atan2(dcm(1,0), dcm(0,0));
        return eul;
}
// NonHolonomic Update
void CoreNav::NonHolonomic(const CoreNav::Vector3 vel, const CoreNav::Vector3 att, const CoreNav::Vector3 llh, CoreNav::Vector15 errorStates, Eigen::MatrixXd P, CoreNav::Vector3 omega_b_ib)
{
        CoreNav::Matrix3 Cnb = CoreNav::eul_to_dcm(att[0],att[1],att[2]);
        CoreNav::Matrix3 ins_vel_ss;         //ins_vel_ skew symmetric
        // ins_vel_ss = CoreNav::skew_symm(omega_b_ib);
        // CoreNav::Vector3 Val_H21(0,cos(ins_att_(0)),sin(ins_att_(0)));
        CoreNav::Vector3 z_holo1;
        CoreNav::Vector3 z_holo2;
        z_holo.row(0) = -eye3.row(1)*(Cnb*vel-CoreNav::skew_symm(omega_b_ib)*(-0.272*(eye3.col(0))));
        z_holo.row(1) = -eye3.row(2)*(Cnb*vel-CoreNav::skew_symm(omega_b_ib)*(-0.272*(eye3.col(0))));

        H_holo.row(0)<<zeros3.row(0), -eye3.row(1)*Cnb, zeros3.row(0), zeros3.row(0),zeros3.row(0);
        H_holo.row(1)<<zeros3.row(0), -eye3.row(2)*Cnb,  zeros3.row(0), H24_.transpose(), zeros3.row(0);


        K_holo = P_ * H_holo.transpose() * (H_holo * P_ * H_holo.transpose() + R_holo).inverse();

        error_states_ = error_states_ + K_holo* (z_holo  - H_holo * error_states_);

        ins_att_ = CoreNav::dcm_to_eul((Eigen::MatrixXd::Identity(3,3)- CoreNav::skew_symm(error_states_.segment(0,3)))*Cnb.transpose());
        ins_vel_ = ins_vel_ - error_states_.segment(3,3);
        ins_pos_ = ins_pos_ - error_states_.segment(6,3);

        error_states_(0)=0.0;
        error_states_(1)=0.0;
        error_states_(2)=0.0;

        error_states_(3)=0.0;
        error_states_(4)=0.0;
        error_states_(5)=0.0;

        error_states_(6)=0.0;
        error_states_(7)=0.0;
        error_states_(8)=0.0;

        P_=(Eigen::MatrixXd::Identity(15,15) - K_holo * H_holo) * P_ * ( Eigen::MatrixXd::Identity(15,15) - K_holo * H_holo ).transpose() + K_holo * R_holo * K_holo.transpose();

        return;
}


// Zero vel. update
void CoreNav::zupt(const CoreNav::Vector3 vel, const CoreNav::Vector3 att, const CoreNav::Vector3 llh, CoreNav::Vector15 errorStates, Eigen::MatrixXd P)
{
        CoreNav::Matrix3 Cnb = CoreNav::eul_to_dcm(att[0],att[1],att[2]);

        CoreNav::Vector3 z_zupt;
        z_zupt = -vel;

        K_zupt = P_ * H_zupt.transpose() * (H_zupt * P_ * H_zupt.transpose() + R_zupt).inverse();

        error_states_ = error_states_ + K_zupt * (z_zupt  - H_zupt * error_states_);

        ins_att_ = CoreNav::dcm_to_eul((Eigen::MatrixXd::Identity(3,3)- CoreNav::skew_symm(error_states_.segment(0,3)))*Cnb.transpose());
        ins_vel_ = ins_vel_ - error_states_.segment(3,3);
        ins_pos_ = ins_pos_ - error_states_.segment(6,3);

        error_states_(0)=0.0;
        error_states_(1)=0.0;
        error_states_(2)=0.0;

        error_states_(3)=0.0;
        error_states_(4)=0.0;
        error_states_(5)=0.0;

        error_states_(6)=0.0;
        error_states_(7)=0.0;
        error_states_(8)=0.0;

        P_=(Eigen::MatrixXd::Identity(15,15) - K_zupt * H_zupt) * P_ * ( Eigen::MatrixXd::Identity(15,15) - K_zupt * H_zupt ).transpose() + K_zupt * R_zupt * K_zupt.transpose();

        return;
}

// Zero ang. update
void CoreNav::zaru(const CoreNav::Vector3 vel, const CoreNav::Vector3 att, const CoreNav::Vector3 llh, CoreNav::Vector15 errorStates, Eigen::MatrixXd P, const CoreNav::Vector3 omega_b_ib)
{

        CoreNav::Matrix3 Cnb = CoreNav::eul_to_dcm(att[0],att[1],att[2]);

        CoreNav::Vector3 z_zaru;
        z_zaru = -omega_b_ib.transpose();

        K_zaru = P_ * H_zaru.transpose() * (H_zaru * P_ * H_zaru.transpose() + R_zaru).inverse();

        error_states_ = error_states_ + K_zaru * (z_zaru  - H_zaru * error_states_);

        ins_att_ = CoreNav::dcm_to_eul((Eigen::MatrixXd::Identity(3,3)- CoreNav::skew_symm(error_states_.segment(0,3)))*Cnb.transpose());
        ins_vel_ = ins_vel_ - error_states_.segment(3,3);
        ins_pos_ = ins_pos_ - error_states_.segment(6,3);

        error_states_(0)=0.0;
        error_states_(1)=0.0;
        error_states_(2)=0.0;

        error_states_(3)=0.0;
        error_states_(4)=0.0;
        error_states_(5)=0.0;

        error_states_(6)=0.0;
        error_states_(7)=0.0;
        error_states_(8)=0.0;

        P_=(Eigen::MatrixXd::Identity(15,15) - K_zaru * H_zaru) * P_ * ( Eigen::MatrixXd::Identity(15,15) - K_zaru * H_zaru ).transpose() + K_zaru * R_zaru * K_zaru.transpose();
        return;
}


CoreNav::Matrix CoreNav::insErrorStateModel_LNF(double R_EPlus, double R_N, CoreNav::Vector3 insLLH, CoreNav::Vector3 insVel, double dt, CoreNav::Matrix3 CbnPlus, double omega_ie,CoreNav::Vector3 omega_n_in,CoreNav::Vector3 f_ib_b,double gravity)
{
        double geoLat= atan2(INS::t_const*sin(insLLH(0)*180.0/INS::PI), cos(insLLH(0)*180.0/INS::PI));
        double rGeoCent  = pow(( pow(INS::Ro,2.0) /( 1.0 + (1.0/(pow(( 1.0 - INS::flat ),2.0)) - 1.0)*pow(sin(geoLat),2.0))),(1.0/2.0));
        double g0 = 9.780318*( 1.0 + (5.3024e-3)*pow(sin(insLLH(0)),2.0) - (5.9e-6)*pow(sin(2*insLLH(0)),2.0) );

        CoreNav::Matrix3 F11;
        F11= CoreNav::skew_symm(-omega_n_in);

        CoreNav::Matrix3 F12;
        F12.row(0)<<0.0, -1.0/(R_EPlus+insLLH(2)), 0.0; //height
        F12.row(1)<<1.0/(R_N+insLLH(2)), 0.0, 0.0; //height
        F12.row(2)<<0.0, tan(insLLH(0))/(R_EPlus+insLLH(2)), 0.0; //lat/height

        CoreNav::Matrix3 F13;
        F13.row(0)<<omega_ie*sin(insLLH(0)), 0.0, insVel(1)/pow((R_EPlus+insLLH(2)),2.0);
        F13.row(1)<<0.0, 0.0, -insVel(0)/pow((R_N+insLLH(2)),2.0);
        F13.row(2)<<omega_ie * cos(insLLH[0]) + insVel[1] / ((R_EPlus + insLLH[2]) *(cos(insLLH[0])* cos(insLLH[0]))),0.0,-insVel[1] * tan(insLLH[0]) / ((R_EPlus + insLLH[2]) * (R_EPlus + insLLH[2]));

        CoreNav::Matrix3 F21 = (-1.0)*CoreNav::skew_symm(CbnPlus*(f_ib_b));

        CoreNav::Matrix3 F22;
        F22.row(0)<< insVel[2] / (R_N + insLLH[2]), -(2.0 * insVel[1] * tan(insLLH[0]) / (R_EPlus + insLLH[2])) - 2.0 *omega_ie * sin(insLLH[0]),insVel[0] / (R_N + insLLH[2]);
        F22.row(1)<< insVel[1] * tan(insLLH[0]) / (R_EPlus + insLLH[2]) + 2.0 * omega_ie *sin(insLLH[0]),(insVel[0] * tan(insLLH[0]) + insVel[2]) / (R_EPlus + insLLH[2]),insVel[1] / (R_EPlus + insLLH[2]) + 2.0 * omega_ie *cos(insLLH[0]);
        F22.row(2)<< -2.0 * insVel[0] / (R_N + insLLH[2]),-2.0 * (insVel[1] / (R_EPlus + insLLH[2])) - 2.0 * omega_ie * cos(insLLH[0]),0.0;

        CoreNav::Matrix3 F23;
        F23.row(0)<< -(insVel[1] * insVel[1] * ((1.0 / cos(insLLH[0])) *(1.0 / cos(insLLH[0]))) / (R_EPlus + insLLH[2])) - 2.0 *insVel[1] * omega_ie * cos(insLLH[0]), 0.0,insVel[1] * insVel[1] * tan(insLLH[0]) / ((R_EPlus + insLLH[2]) * (R_EPlus + insLLH[2])) - insVel[0] * insVel[2] / ((R_N + insLLH[2]) * (R_N + insLLH[2]));
        F23.row(1)<< (insVel[0] * insVel[1] * ((1.0 / cos(insLLH[0])) * (1.0 / cos(insLLH[0]))) / (R_EPlus + insLLH[2]) + 2.0 *insVel[0] * omega_ie * cos(insLLH[0])) - 2.0 * insVel[2] * omega_ie * sin(insLLH[0]),0.0, -((insVel[0] * insVel[1] * tan(insLLH[0]) + insLLH[1] * insLLH[2]) / ((R_EPlus + insLLH[2]) * (R_EPlus + insLLH[2])));
        F23.row(2)<< 2.0 * insVel[1] * omega_ie * sin(insLLH[0]), 0.0, (insVel[1] * insVel[1] / ((R_EPlus + insLLH[2]) * (R_EPlus + insLLH[2])) + insVel[0] * insVel[0] / ((R_N + insLLH[2]) *(R_N + insLLH[2]))) - 2.0 * g0 / rGeoCent;

        CoreNav::Matrix3 F32;
        F32.row(0)<<(1.0)/(R_N+insLLH(2)), 0.0, 0.0;
        F32.row(1)<<0.0, (1.0)/((R_EPlus+insLLH(2))*cos(insLLH(0))), 0.0;
        F32.row(2)<<0.0, 0.0, -1.0;

        CoreNav::Matrix3 F33;
        F33.row(0)<<0.0, 0.0, -insVel(0)/pow((R_N+insLLH(2)),2.0);
        F33.row(1)<<(insVel(1)*sin(insLLH(0)))/((R_EPlus+insLLH(2))*pow(cos(insLLH(0)),2.0)), 0.0, -insVel(1)/(pow((R_EPlus+insLLH(2)),2.0)*cos(insLLH(0)));
        F33.row(2)<<0.0, 0.0, 0.0;

        Eigen::Matrix3d PHI11 = Eigen::Matrix3d::Identity(3,3)+F11*dt;
        Eigen::Matrix3d PHI12 = F12*dt;
        Eigen::Matrix3d PHI13 = F13*dt;
        Eigen::Matrix3d PHI15 = CbnPlus*dt;
        Eigen::Matrix3d PHI21 = F21*dt;
        Eigen::Matrix3d PHI22 = Eigen::Matrix3d::Identity()+F22*dt;
        Eigen::Matrix3d PHI23 = F23*dt;
        Eigen::Matrix3d PHI24 = CbnPlus*dt;
        Eigen::Matrix3d PHI32 = F32*dt;
        Eigen::Matrix3d PHI33 = Eigen::Matrix3d::Identity()+F33*dt;

//%Eq:14.72
        CoreNav::Matrix STM(15,15);

        STM<<PHI11, PHI12, PHI13, Eigen::Matrix3d::Zero(3,3),PHI15,
        PHI21, PHI22, PHI23, PHI24, Eigen::Matrix3d::Zero(3,3),
        Eigen::Matrix3d::Zero(3,3), PHI32, PHI33, Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),
        Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Identity(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),
        Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Identity(3,3);

        return STM;
}

// TODO:: This needs to read in the values specifed in parameters.yaml
CoreNav::Matrix CoreNav::calc_Q(double R_N, double R_E, CoreNav::Vector3 insLLH, double dt, CoreNav::Matrix3 CbnPlus,CoreNav::Vector3 f_ib_b)
{
        CoreNav::Matrix3 F21 = (-1.0)*CoreNav::skew_symm(CbnPlus*(f_ib_b));
        Eigen::Matrix3d T_rn_p;
        T_rn_p.row(0)<<1.0/(R_N+insLLH(2)),0.0,0.0;
        T_rn_p.row(1)<<0.0,1.0/((R_E+insLLH(2))*cos(insLLH(0))),0.0;
        T_rn_p.row(2)<<0.0,0.0,-1.0;
        double gg=9.80665;

// TODO: Revisit here!! it should be for adis 16488, these are for 95
        double sig_gyro_inRun = 1.6*INS::PI/180/3600; //rad/s
        double sig_ARW = .1*(INS::PI/180)*sqrt(3600)/3600;; //rad

        double sig_accel_inRun = (3.2e-6)*gg; // m/s
        double sig_VRW = 0.008*sqrt(3600)/3600; //m/s

//following 14.2.6 of Groves pp 592
        double Srg= pow(sig_ARW,2)*dt;
        double Sra= pow(sig_VRW,2)*dt;

        double Sbad=pow(sig_accel_inRun,2)/dt;
        double Sbgd=pow(sig_gyro_inRun,2)/dt;

        Eigen::Matrix3d Q11 = (Srg*dt+(1.0/3.0)*Sbgd*pow(dt,3.0))*Eigen::Matrix3d::Identity(3,3);
        Eigen::Matrix3d Q21= ((1.0/2.0)*Srg*pow(dt,2)+(1.0/4.0)*Sbgd*pow(dt,4.0))*F21;
        Eigen::Matrix3d Q12=Q21.transpose();
        Eigen::Matrix3d Q31= ((1.0/3.0)*Srg*pow(dt,3.0)+(1.0/5.0)*Sbgd*pow(dt,5.0))*T_rn_p*F21;
        Eigen::Matrix3d Q13=Q31.transpose();
        Eigen::Matrix3d Q14=Eigen::Matrix3d::Zero(3,3);
        Eigen::Matrix3d Q15=(1.0/2.0)*Sbgd*pow(dt,2.0)*CbnPlus;
        Eigen::Matrix3d Q22= (Sra*dt+(1.0/3.0)*Sbad*pow(dt,3.0))*Eigen::Matrix3d::Identity(3,3)+((1.0/3.0)*Srg*pow(dt,3.0)+(1.0/5.0)*Sbgd*pow(dt,5.0))*F21*F21.transpose();
        Eigen::Matrix3d Q32= ((1.0/2.0)*Sra*pow(dt,2.0)+(1.0/4.0)*Sbad*pow(dt,4))*T_rn_p+((1.0/4.0)*Srg*pow(dt,4.0)+(1.0/6.0)*Sbgd*pow(dt,6.0))*T_rn_p*F21*F21.transpose();
        Eigen::Matrix3d Q23=Q32.transpose();
        Eigen::Matrix3d Q24= (1.0/2.0)*Sbad*pow(dt,2.0)*CbnPlus;
        Eigen::Matrix3d Q25= (1.0/3.0)*Sbgd*pow(dt,3.0)*F21*CbnPlus;
        Eigen::Matrix3d Q33= ((1.0/3.0)*Sra*pow(dt,3.0) + (1.0/5.0)*Sbad*pow(dt,5.0))*(T_rn_p*T_rn_p)+((1.0/5.0)*Srg*pow(dt,5.0)+(1.0/7.0)*Sbgd*pow(dt,7.0))*T_rn_p*(F21*F21.transpose())*T_rn_p;
        Eigen::Matrix3d Q34=(1.0/3.0)*Sbad*pow(dt,3.0)*T_rn_p*CbnPlus;
        Eigen::Matrix3d Q35=(1.0/4.0)*Sbgd*pow(dt,4.0)*T_rn_p*F21*CbnPlus;
        Eigen::Matrix3d Q41= Eigen::Matrix3d::Zero(3,3);
        Eigen::Matrix3d Q42= (1.0/2.0)*Sbad*pow(dt,2.0)*(CbnPlus.transpose());
        Eigen::Matrix3d Q43=Q34.transpose();
        Eigen::Matrix3d Q44=Sbad*dt*Eigen::Matrix3d::Identity(3,3);
        Eigen::Matrix3d Q45=Eigen::Matrix3d::Zero(3,3);
        Eigen::Matrix3d Q51= (1.0/2.0)*Sbgd*pow(dt,2.0)*(CbnPlus.transpose());
        Eigen::Matrix3d Q52=(1.0/3.0)*Sbgd*pow(dt,3.0)*F21.transpose()*(CbnPlus.transpose());
        Eigen::Matrix3d Q53=Q35.transpose();
        Eigen::Matrix3d Q54=Eigen::Matrix3d::Zero(3,3);
        Eigen::Matrix3d Q55= Sbgd*dt*Eigen::Matrix3d::Identity(3,3);

        CoreNav::Matrix Q(15,15);
        Q<<Q11,Q12,Q13,Q14,Q15,
        Q21,Q22,Q23,Q24,Q25,
        Q31,Q32,Q33,Q34,Q35,
        Q41,Q42,Q43,Q44,Q45,
        Q51,Q52,Q53,Q54,Q55;
        return Q;
}
