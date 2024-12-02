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

}

void LieDownController::update(const ros::Time& time, const ros::Duration& period) {

    if(_percent_4<1)
    {
        // std::cout<<"The example is running! "<<std::endl;
    }
    if((_percent_4 == 1) && ( done == false))
    {
        std::cout<<"The example is done! "<<std::endl;
        std::cout<<std::endl;
        done = true;
    }

    motiontime++;
    if(motiontime>=500)
    {
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
        if (_percent_1 < 1)
        {
            // ROS_INFO("----------------------Phase 1------------------------");
            for (int j = 0; j < 12; j++)
            {
                auto posDes = (1 - _percent_1) * _startPos[j] + _percent_1 * _targetPos_1[j];
                hybridJointHandles_[j].setCommand(posDes, 0, Kp, Kd, 0);
                // ROS_INFO_STREAM("Joint " <<  j << " : " << posDes);
            }
        
        }
        if ((_percent_1 == 1)&&(_percent_2 < 1))
        {
            _percent_2 += (float)1 / _duration_2;
            _percent_2 = _percent_2 > 1 ? 1 : _percent_2;

            // ROS_INFO("----------------------Phase 2------------------------");
            for (int j = 0; j < 12; j++)
            {
                auto posDes = (1 - _percent_2) * _targetPos_1[j] + _percent_2 * _targetPos_2[j];
                hybridJointHandles_[j].setCommand(posDes, 0, Kp, Kd, 0);
                // ROS_INFO_STREAM("Joint " <<  j << " : " << posDes);
            }
        }

        if ((_percent_1 == 1)&&(_percent_2 == 1)&&(_percent_3<1))
        {
            _percent_3 += (float)1 / _duration_3;
            _percent_3 = _percent_3 > 1 ? 1 : _percent_3;

            // ROS_INFO("----------------------Phase 3------------------------");

            for (int j = 0; j < 12; j++)
            {
                auto posDes = _targetPos_2[j];
                hybridJointHandles_[j].setCommand(posDes, 0, Kp, Kd, 0);
                // ROS_INFO_STREAM("Joint " <<  j << " : " << posDes);
            }
        }
        if ((_percent_1 == 1)&&(_percent_2 == 1)&&(_percent_3==1)&&((_percent_4<=1)))
        {
            _percent_4 += (float)1 / _duration_4;
            _percent_4 = _percent_4 > 1 ? 1 : _percent_4;
         
            ROS_INFO("----------------------------------------------");
            for (int j = 0; j < 12; j++)
            {
                auto posDes = (1 - _percent_4) * _targetPos_2[j] + _percent_4 * _targetPos_3[j];
                hybridJointHandles_[j].setCommand(posDes, 0, Kp, Kd, 0);
                auto pos = hybridJointHandles_[j].getPosition();
                if(posDes - pos > 0.05)
                {
                    ROS_INFO_STREAM("Joint " <<  j << " : Error: " << posDes - pos << " Des: " << posDes << " Act: " << pos);
                }
                // ROS_INFO_STREAM("Joint " <<  j << " : Error: " << posDes - pos << " Des: " << posDes << " Act: " << pos);
            }
        }
    }
}

LieDownController::~LieDownController() {
  controllerRunning_ = false;
  std::cout << "LieDownController is destructed." << std::endl;
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LieDownController, controller_interface::ControllerBase)
