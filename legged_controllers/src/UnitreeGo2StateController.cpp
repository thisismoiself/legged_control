#include <pinocchio/fwd.hpp> // forward declarations must be included first.

#include "legged_controllers/UnitreeGo2StateController.h"

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

#include <hardware_interface/joint_state_interface.h>

namespace unitree_go2_state_controller
{
    bool UnitreeGo2StateController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh)
    {
        bool verbose = false;

        // Hardware interface
        auto *jointInterface = robot_hw->get<hardware_interface::JointStateInterface>();
        std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                             "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
        for (const auto &joint_name : joint_names)
        {
            jointHandles_.push_back(jointInterface->getHandle(joint_name));
        }
        auto *contactInterface = robot_hw->get<legged::ContactSensorInterface>();
        for (const auto &name : {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"})
        {
            contactHandles_.push_back(contactInterface->getHandle(name));
        }
        imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");


        for (unsigned i = 0; i < 12; i++)
        {
            jointHandles_.push_back(jointInterface->getHandle(joint_names[i]));
            joint_state_publisher->msg_.name.push_back(joint_names[i]);
            joint_state_publisher->msg_.position.push_back(0.0);
            joint_state_publisher->msg_.velocity.push_back(0.0);
            joint_state_publisher->msg_.effort.push_back(0.0);
        }

        return true;
    }

    void UnitreeGo2StateController::starting(const ros::Time &time)
    {
        ROS_INFO_STREAM("Starting UnitreeGo2StateController ...");
    }

    void UnitreeGo2StateController::stopping(const ros::Time &time)
    {
        ROS_INFO_STREAM("Stopping UnitreeGo2StateController ...");
    }

    void UnitreeGo2StateController::update(const ros::Time &time, const ros::Duration &period)
    {
        /* Joint States */
        if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
        {

            // try to publish
            if (joint_state_publisher->trylock())
            {
                // we're actually publishing, so increment time
                last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

                // populate joint state message:
                // - fill only joints that are present in the JointStateInterface, i.e. indices [0, num_hw_joints_)
                // - leave unchanged extra joints, which have static values, i.e. indices from num_hw_joints_ onwards
                joint_state_publisher->msg_.header.stamp = time;
                for (unsigned i = 0; i < num_hw_joints_; i++)
                {
                    joint_state_publisher->msg_.position[i] = jointHandles_[i].getPosition();
                    joint_state_publisher->msg_.velocity[i] = jointHandles_[i].getVelocity();
                    joint_state_publisher->msg_.effort[i] = jointHandles_[i].getEffort();
                }
                joint_state_publisher->unlockAndPublish();
            }
        }
        /* IMU */
        if (publish_rate_ > 0.0 && last_publish_time_imu_ + ros::Duration(1.0 / publish_rate_) < time)
        {
            imu_publisher->msg_.header.stamp = time;
            imu_publisher->msg_.header.frame_id = "imu_link";

            imu_publisher->msg_.angular_velocity.x = imuSensorHandle_.getAngularVelocity()[0];
            imu_publisher->msg_.angular_velocity.y = imuSensorHandle_.getAngularVelocity()[1];
            imu_publisher->msg_.angular_velocity.z = imuSensorHandle_.getAngularVelocity()[2];
            imu_publisher->msg_.linear_acceleration.x = imuSensorHandle_.getLinearAcceleration()[0];
            imu_publisher->msg_.linear_acceleration.y = imuSensorHandle_.getLinearAcceleration()[1];
            imu_publisher->msg_.linear_acceleration.z = imuSensorHandle_.getLinearAcceleration()[2];
            imu_publisher->msg_.orientation.x = imuSensorHandle_.getOrientation()[0];
            imu_publisher->msg_.orientation.y = imuSensorHandle_.getOrientation()[1];
            imu_publisher->msg_.orientation.z = imuSensorHandle_.getOrientation()[2];
            imu_publisher->msg_.orientation.w = imuSensorHandle_.getOrientation()[3];

            for (size_t i = 0; i < 9; ++i)
            {
                imu_publisher->msg_.angular_velocity_covariance[i] = imuSensorHandle_.getLinearAcceleration()[i];
                imu_publisher->msg_.linear_acceleration_covariance[i] = imuSensorHandle_.getLinearAccelerationCovariance()[i];
                imu_publisher->msg_.orientation_covariance[i] = imuSensorHandle_.getOrientationCovariance()[i];
            }
        }
        /* Contact */
        if (publish_rate_ > 0.0 && last_publish_time_contact_ + ros::Duration(1.0 / publish_rate_) < time)
        {   
            
            for (size_t i = 0; i < contacts.size(); ++i)
            {
                contactFlag[i] = contactHandles_[i].isContact();
            }
        }
    }

    void UnitreeGo2StateController::updateStateEstimation(const ros::Time &time, const ros::Duration &period)
    {
        for (size_t i = 0; i < contacts.size(); ++i)
        {
            contactFlag[i] = contactHandles_[i].isContact();
        }
        for (size_t i = 0; i < 4; ++i)
        {
            quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
        }
        for (size_t i = 0; i < 3; ++i)
        {
            angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
            linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
        }
        for (size_t i = 0; i < 9; ++i)
        {
            orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
            angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
            linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
        }

        stateEstimate_->updateJointStates(jointPos, jointVel);
        stateEstimate_->updateContact(contactFlag);
        stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
        measuredRbdState_ = stateEstimate_->update(time, period);
        currentObservation_.time += period.toSec();
        scalar_t yawLast = currentObservation_.state(9);
        currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
        currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
        currentObservation_.mode = stateEstimate_->getMode();
    }

    UnitreeGo2StateController::~UnitreeGo2StateController()
    {
        ROS_INFO_STREAM("Stopping UnitreeGo2StateController ...");
    }

} // namespace legged

PLUGINLIB_EXPORT_CLASS(unitree_go2_state_controller::UnitreeGo2StateController, controller_interface::ControllerBase)
