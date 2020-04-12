/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/canbus/vehicle/vehicle_controller.h"

#include "cyber/common/log.h"

namespace apollo {
namespace canbus {

//一个using声明一次只引入一个命名空间成员。
using common::ErrorCode;
using control::ControlCommand;

//返回驾驶模式
Chassis::DrivingMode VehicleController::driving_mode() {
  std::lock_guard<std::mutex> lock(mode_mutex_); //构造并锁定
  return driving_mode_;
}

//传入DrivingMode类的引用，并为驾驶模式赋值（新值覆盖旧值）
void VehicleController::set_driving_mode(
    const Chassis::DrivingMode &driving_mode) {
  std::lock_guard<std::mutex> lock(mode_mutex_); //构造并锁定
  driving_mode_ = driving_mode;
}

//返回设置驾驶模式时的故障码
ErrorCode VehicleController::SetDrivingMode(
    const Chassis::DrivingMode &driving_mode) {
  ////如果设置的驾驶模式是紧急模式，显示信息并返回故障码
  if (driving_mode == Chassis::EMERGENCY_MODE) {
    AINFO << "Can't set vehicle to EMERGENCY_MODE driving mode.";
    return ErrorCode::CANBUS_ERROR; 
  }

  // vehicle in emergency mode only response to manual mode to reset.
  //车辆在紧急模式下只能响应手动模式才能重置。
  //如果设置模式是紧急模式并且不是手动模式，显示信息并返回故障码
  if (driving_mode_ == Chassis::EMERGENCY_MODE &&
      driving_mode != Chassis::COMPLETE_MANUAL) {
    AINFO
        << "Vehicle in EMERGENCY_MODE, only response to COMPLETE_MANUAL mode.";
    AINFO << "Only response to RESET ACTION.";
    return ErrorCode::CANBUS_ERROR;
  }

  // if current mode is same as previous, no need to set.
  //如果新的模式和之前的模式一样，不需要设置,返回OK故障码
  if (driving_mode_ == driving_mode) {
    return ErrorCode::OK;
  }

  //driving_mode为真时，执行case语句返回故障码
  switch (driving_mode) {
    case Chassis::COMPLETE_AUTO_DRIVE: {
      //如果可以自动驾驶模式不等于OK，也就是可以自动驾驶模式判断为假时为非OK故障码，执行if语句.
      //如果可以自动驾驶模式判断为真时为OK故障码，不执行if语句。
      if (EnableAutoMode() != ErrorCode::OK) {
        AERROR << "Failed to enable auto mode."; //不能可以自动驾驶模式
        return ErrorCode::CANBUS_ERROR;
      }
      break; //break语句的作用是：结束当前正在执行的循环（for、while、do…while）或多路分支（switch）程序结构，转而执行这些结构后面的语句。
    }
    case Chassis::COMPLETE_MANUAL: {
      //如果不可以自动驾驶不等于OK，也就是不可以自动驾驶判断为假时（可以理解为可以自动驾驶）为非OK故障码，执行if语句
      //如果不可以自动驾驶模式判断为真时为OK故障码，不执行if语句
      if (DisableAutoMode() != ErrorCode::OK) {
        AERROR << "Failed to disable auto mode."; //不能不可以自动驾驶模式=可以自动驾驶模式
        return ErrorCode::CANBUS_ERROR;
      }
      break;
    }
    case Chassis::AUTO_STEER_ONLY: {
      //可以仅转向模式为假时（不只是可以转向）为非OK故障码，执行if语句
      if (EnableSteeringOnlyMode() != ErrorCode::OK) {
        AERROR << "Failed to enable steering only mode."; //不能只转向模式
        return ErrorCode::CANBUS_ERROR;
      }
      break;
    }
    case Chassis::AUTO_SPEED_ONLY: {
      //可以只自动速度模式为假时为非OK故障码，执行if语句
      if (EnableSpeedOnlyMode() != ErrorCode::OK) {
        AERROR << "Failed to enable speed only mode"; //不能启用仅速度模式
        return ErrorCode::CANBUS_ERROR;
      }
      break;
    }
    //其他情况，退出条件语句
    default:
      break;
  }
  return ErrorCode::OK; //返回OK故障码
}

//返回更新控制器的故障码
ErrorCode VehicleController::Update(const ControlCommand &control_command) {
  //没有初始化为真
  if (!is_initialized_) {
    AERROR << "Controller is not initialized."; //控制器没有被初始化
    return ErrorCode::CANBUS_ERROR;
  }

  // Execute action to transform driving mode
  //执行动作以改变驾驶模式
  //如果有便笺且有动作
  if (control_command.has_pad_msg() && control_command.pad_msg().has_action()) {
    AINFO << "Canbus received pad msg: " //canbus收到便笺
          << control_command.pad_msg().ShortDebugString(); //调试信息
    Chassis::DrivingMode mode = Chassis::COMPLETE_MANUAL; //驾驶模式为完全手动模式
    //有动作时为真，执行switch语句
    switch (control_command.pad_msg().action()) {
      //动作为开始时
      case control::DrivingAction::START: {
        mode = Chassis::COMPLETE_AUTO_DRIVE; //驾驶模式为完全自动驾驶
        break;
      }
      case control::DrivingAction::STOP: //动作为结束
      //动作为重置
      case control::DrivingAction::RESET: {
        // In COMPLETE_MANUAL mode
        //在完全手动驾驶模式
        break;
      }
      default: {
        AERROR << "No response for this action."; //没有反应
        break;
      }
    }
    SetDrivingMode(mode); //设置驾驶模式为mode
  }

  //如果驾驶模式是完全自动驾驶或者仅速度驾驶时为真
  if (driving_mode_ == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode_ == Chassis::AUTO_SPEED_ONLY) {
    Gear(control_command.gear_location()); //换挡状态
    Throttle(control_command.throttle()); //节气门控制
    Acceleration(control_command.acceleration()); //加速度控制
    Brake(control_command.brake()); //刹车控制
    SetEpbBreak(control_command); //设置电子驻车
    SetLimits(); //设置限制
  }

  //如果驾驶模式时完全自动驾驶或者仅转向驾驶时为真
  if (driving_mode_ == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode_ == Chassis::AUTO_STEER_ONLY) {
    const double steering_rate_threshold = 1.0; //转向角速度阈值
    //判断转向角速度和阈值
    if (control_command.steering_rate() > steering_rate_threshold) {
      //Steer(角度，角速度)
      Steer(control_command.steering_target(), control_command.steering_rate());
    } else {
      //Steer(角度)
      Steer(control_command.steering_target());
    }
  }
  //如果驾驶模式为完全自动驾驶或者自动速度或自动转向中的一种且有控制命令信号时为真
  if ((driving_mode_ == Chassis::COMPLETE_AUTO_DRIVE ||
       driving_mode_ == Chassis::AUTO_SPEED_ONLY ||
       driving_mode_ == Chassis::AUTO_STEER_ONLY) &&
      control_command.has_signal()) {
    SetHorn(control_command); //喇叭
    SetTurningSignal(control_command); //转向灯信号
    SetBeam(control_command); //远近光灯
  }

  return ErrorCode::OK; //返回无故障
}

}  // namespace canbus
}  // namespace apollo
