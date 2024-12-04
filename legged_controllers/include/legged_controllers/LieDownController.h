//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>
namespace legged {

class LieDownController : public controller_interface::MultiInterfaceController<HybridJointInterface> {
 public:
  LieDownController() = default;
  ~LieDownController() override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override;
  void reset();

 protected:

  // Interface
  std::vector<HybridJointHandle> hybridJointHandles_;

 private:
  std::atomic_bool controllerRunning_{};

    float Kp = 60.0;
    float Kd = 5.0;
    int motiontime = 0;
    float dt = 0.002; // 0.001~0.01
    

    //LF LH RF RH
    float _targetPos_1[12] = {0.0, 1.26, -2.70, 0.33, 1.26, -2.70,
                              0.0, 1.26, -2.70, -0.33, 1.26, -2.70};

    float _startPos[12];
    float _duration_1 = 500;   
    float _duration_2 = 500; 
    float _duration_3 = 500; 
    float _percent_1 = 0;    
    float _percent_2 = 0;    
    float _percent_3 = 0;    

    bool firstRun = true;
    bool done = false;

};



}  // namespace legged
