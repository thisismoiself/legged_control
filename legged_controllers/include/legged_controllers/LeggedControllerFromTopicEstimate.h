//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_estimation/FromTopicEstimate.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/WbcBase.h>

#include "legged_controllers/LeggedController.h"
#include "legged_controllers/SafetyChecker.h"
#include "legged_controllers/visualization/LeggedSelfCollisionVisualization.h"

namespace legged
{
  using namespace ocs2;
  using namespace legged_robot;

  class LeggedControllerFromTopicEstimate : public LeggedController
  {
  public:
    LeggedControllerFromTopicEstimate() = default;
    ~LeggedControllerFromTopicEstimate() override;

    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
    // void update(const ros::Time& time, const ros::Duration& period) override;
    // void starting(const ros::Time& time) override;
    // void stopping(const ros::Time& /*time*/) override;
  
   protected:
    virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);
    virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  private:
    std::thread mpcThread_;
    std::atomic_bool controllerRunning_{}, mpcRunning_{};
    benchmark::RepeatedTimer mpcTimer_;
    benchmark::RepeatedTimer wbcTimer_;
    ros::NodeHandle nh_;
    bool isSimulation_{false};
    ros::Publisher imuPublisher_, jointStatePublisher_;
  };

} // namespace legged
