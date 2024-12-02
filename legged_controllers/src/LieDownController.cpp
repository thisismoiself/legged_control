//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_controllers/LieDownController.h"

#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged {
bool LieDownController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  bool verbose = false;


  // Hardware interface
  auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                       "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  for (const auto& joint_name : joint_names) {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  }

  return true;
}

void LieDownController::starting(const ros::Time& time) {

    ROS_INFO_STREAM("Starting LieDownController ...");
    reset();

}

void LieDownController::reset() {
    ROS_INFO("Resetting LieDownController ...");
    Kp = 60.0;
    Kd = 5.0;
    motiontime = 0;
    dt = 0.002; // 0.001~0.01
    
    _startPos[12];
    _duration_1 = 500;   
    _duration_2 = 500; 
    _percent_1 = 0;    
    _percent_2 = 0;    

    bool firstRun = true;
}

void LieDownController::update(const ros::Time& time, const ros::Duration& period) {

    motiontime++;

    if(firstRun)
    {
        for(int i = 0; i < 12; i++)
        {
            _startPos[i] = hybridJointHandles_[i].getPosition(); // Read initial joint positions
        }
        firstRun = false;
    }

    _percent_1 += (float)1 / _duration_1;
    _percent_1 = _percent_1 > 1 ? 1 : _percent_1;
    if (_percent_1 < 1) // moving down
    {
        for (int j = 0; j < 12; j++)
        {
            auto posDes = (1 - _percent_1) * _startPos[j] + _percent_1 * _targetPos_1[j];
            hybridJointHandles_[j].setCommand(posDes, 0, Kp, Kd, 0);
        }
    }
    if ((_percent_1 == 1)&&(_percent_2 <= 1)) // lying down
    {
        _percent_2 += (float)1 / _duration_2;
        _percent_2 = _percent_2 > 1 ? 1 : _percent_2;

        for (int j = 0; j < 12; j++)
        {
            auto posDes = _targetPos_1[j];
            hybridJointHandles_[j].setCommand(posDes, 0, Kp, Kd, 0);
        }
    }
}

LieDownController::~LieDownController() {
  controllerRunning_ = false;
  std::cout << "LieDownController is destructed." << std::endl;
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LieDownController, controller_interface::ControllerBase)
