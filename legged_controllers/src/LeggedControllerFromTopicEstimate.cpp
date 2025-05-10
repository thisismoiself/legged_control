//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp> // forward declarations must be included first.

#include "legged_controllers/LeggedControllerFromTopicEstimate.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopicEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

namespace legged
{
  bool LeggedControllerFromTopicEstimate::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh)
  {
    ROS_INFO_STREAM("Initializing LeggedControllerFromTopicEstimate ...");
    nh_ = controller_nh;

    if (nh_.getParam("/is_simulation", isSimulation_) && isSimulation_)
    {
      ROS_INFO_STREAM("LeggedControlerFromTopicEstimate is running in simulation, creating publishers.");
      imuPublisher_ = nh_.advertise<sensor_msgs::Imu>("/base_imu", 1);
      // jointStatePublisher_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
    }
    else
    {
      ROS_INFO_STREAM("LeggedControlerFromTopicEstimate is running on real robot.");
    }

    return LeggedController::init(robot_hw, controller_nh);
  }

  void LeggedControllerFromTopicEstimate::update(const ros::Time &time, const ros::Duration &period)
  {
    LeggedController::update(time, period);

    if (isSimulation_)
    {
      publishMsgs(time);
    }
  }

  void LeggedControllerFromTopicEstimate::publishMsgs(const ros::Time &time)
  {
    /* --- IMU --- */
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = time;
    imu_msg.header.frame_id = "imu_link";

    imu_msg.angular_velocity.x = imuSensorHandle_.getAngularVelocity()[0];
    imu_msg.angular_velocity.y = imuSensorHandle_.getAngularVelocity()[1];
    imu_msg.angular_velocity.z = imuSensorHandle_.getAngularVelocity()[2];
    imu_msg.linear_acceleration.x = imuSensorHandle_.getLinearAcceleration()[0];
    imu_msg.linear_acceleration.y = imuSensorHandle_.getLinearAcceleration()[1];
    imu_msg.linear_acceleration.z = imuSensorHandle_.getLinearAcceleration()[2];
    imu_msg.orientation.x = imuSensorHandle_.getOrientation()[0];
    imu_msg.orientation.y = imuSensorHandle_.getOrientation()[1];
    imu_msg.orientation.z = imuSensorHandle_.getOrientation()[2];
    imu_msg.orientation.w = imuSensorHandle_.getOrientation()[3];

    for (size_t i = 0; i < 9; ++i)
    {
      imu_msg.angular_velocity_covariance[i] = imuSensorHandle_.getAngularVelocityCovariance()[i];
      imu_msg.linear_acceleration_covariance[i] = imuSensorHandle_.getLinearAccelerationCovariance()[i];
      imu_msg.orientation_covariance[i] = imuSensorHandle_.getOrientationCovariance()[i];
    }

    imuPublisher_.publish(imu_msg);
    
    // /* --- Joint State --- */
    // sensor_msgs::JointState joint_state_msg;
    // joint_state_msg.header.stamp = time;
    // joint_state_msg.header.frame_id = "base_link"; // TODO: check if this is correct

    // joint_state_msg.name.resize(hybridJointHandles_.size());
    // joint_state_msg.position.resize(hybridJointHandles_.size());
    // joint_state_msg.velocity.resize(hybridJointHandles_.size());
    // joint_state_msg.effort.resize(hybridJointHandles_.size());

    // for (size_t i = 0; i < hybridJointHandles_.size(); ++i)
    // {
    //   joint_state_msg.name[i] = hybridJointHandles_[i].getName();
    //   joint_state_msg.position[i] = hybridJointHandles_[i].getPosition();
    //   joint_state_msg.velocity[i] = hybridJointHandles_[i].getVelocity();
    //   joint_state_msg.effort[i] = hybridJointHandles_[i].getEffort();
    // }

    // // jointStatePublisher_.publish(joint_state_msg);
  }

  void LeggedControllerFromTopicEstimate::setupStateEstimate(const std::string &taskFile, bool verbose)
  {

    stateEstimate_ = std::make_shared<FromTopicStateEstimate>(leggedInterface_->getPinocchioInterface(),
                                                              leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);

    currentObservation_.time = 0;
  }

} // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedControllerFromTopicEstimate, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerBase)
