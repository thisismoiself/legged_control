#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>


#include <memory>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16MultiArray.h>
#include <controller_interface/multi_interface_controller.h>

namespace unitree_go2_state_controller
{

    class UnitreeGo2StateController : public controller_interface::MultiInterfaceController<hardware_interface::JointStateInterface, hardware_interface::ImuSensorInterface,
    legged::ContactSensorInterface>
    {
    public:
        UnitreeGo2StateController() : publish_rate_(0.0) {}
        // bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;

        void update(const ros::Time& time, const ros::Duration& period) override;
        void starting(const ros::Time& time) override;
        void stopping(const ros::Time& /*time*/) override;

    private:
        /* Handles */
        std::vector<hardware_interface::JointStateHandle> jointHandles_;
        std::vector<legged::ContactSensorHandle> contactHandles_;
        hardware_interface::ImuSensorHandle imuSensorHandle_;

        /* Publishers */
        std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> joint_state_publisher;
        std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Imu>> imu_publisher;
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Int16MultiArray>> contact_publisher;

        ros::Time last_publish_time_joint_state_, last_publish_time_imu_, last_publish_time_contact_, last_publish_time_;
        double publish_rate_;
        unsigned int num_hw_joints_; ///< Number of joints present in the JointStateInterface, excluding extra joints
        bool pub_time_initialized_;

    };

}