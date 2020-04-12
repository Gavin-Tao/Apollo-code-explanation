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

/**
 * @file vehicle_controller.h
 * @brief The class of VehicleController VehicleController类
 */

#pragma once

#include <unordered_map>

#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/control/proto/control_cmd.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

/**
 * @namespace apollo::canbus
 * @brief apollo::canbus
 */
namespace apollo {
namespace canbus {

using ::apollo::drivers::canbus::CanSender;
using ::apollo::drivers::canbus::MessageManager;

/**
 * @class VehicleController
 *
 * @brief This is the interface class of vehicle controller. It defines pure
 * virtual functions, and also some implemented common functions.
 * 这是车辆控制器的接口类。 它定义了纯虚函数，还定义了一些已实现的常用函数。
 */
class VehicleController {
 public:
 //基类声明的虚函数，在派生类中也是虚函数，即使不再使用virtual关键字。
  virtual ~VehicleController() = default;//析构函数

  /**
   * @brief initialize the vehicle controller.  初始化车辆控制器
   * @param params  VehicleParameter类以引用的方式传入
   * @param can_sender a pointer to canbus sender.  can_sender指向canbus发送者的指针。
   * @param message_manager a pointer to the message_manager. message_manager指向message_manager的指针。
   * @return error_code  返回故障码
   */
  //=0标志一个虚函数为纯虚函数
  //纯虚函数用来规范派生类的行为，实际上就是所谓的“接口”。它告诉使用者，我的派生类都会有这个函数。
  virtual common::ErrorCode Init(
      const VehicleParameter &params,
      CanSender<ChassisDetail> *const can_sender,
      MessageManager<ChassisDetail> *const message_manager) = 0;

  /**
   * @brief start the vehicle controller.  启动车辆控制器。
   * @return true if successfully started.  如果成功启动，则返回true。
   */
  virtual bool Start() = 0;

  /**
   * @brief stop the vehicle controller.  停止车辆控制器。
   */
  virtual void Stop() = 0;

  /**
   * @brief calculate and return the chassis.  计算并返回底盘信息
   * @returns a copy of chassis. Use copy here to avoid multi-thread issues. 底盘信息的副本。使用副本来避免多线程问题
   */
  virtual Chassis chassis() = 0;

  /**
   * @brief update the vehicle controller.  更新车辆控制器。
   * @param command the control command   ControlCommand类以引用名称为command的形式传入   command：控制命令
   * @return error_code  返回故障码
   */
  virtual common::ErrorCode Update(const control::ControlCommand &command);

  /**
   * @brief set vehicle to appointed driving mode.  将车辆设置为指定的驾驶模式。
   * @param driving mode to be appointed.  driving代表要指定的模式。 将DrivingMode以driving_mode引用方式传入
   * @return error_code   返回故障码
   */
  virtual common::ErrorCode SetDrivingMode(
      const Chassis::DrivingMode &driving_mode);

 private:
  /*
   * @brief main logical function for operation the car enter or exit the auto
   * driving
   * 汽车进入或退出自动驾驶操作的主要逻辑功能
   */
  virtual void Emergency() = 0;

  virtual common::ErrorCode EnableAutoMode() = 0; //可以自动驾驶
  virtual common::ErrorCode DisableAutoMode() = 0; //不可以自动驾驶
  virtual common::ErrorCode EnableSteeringOnlyMode() = 0; //只可以转向
  virtual common::ErrorCode EnableSpeedOnlyMode() = 0; //只可以速度

  /*
   * @brief NEUTRAL, REVERSE, DRIVE
   * 空挡，倒挡，前进挡
   */
  virtual void Gear(Chassis::GearPosition state) = 0; //换挡

  /*
   * @brief detail function for auto driving brake with new acceleration
   * 具有新加速度的自动驾驶制动器的详细功能
   * acceleration:0.00~99.99, unit:%
   * 加速范围：0.00~99.99%
   */
  virtual void Brake(double acceleration) = 0;

  /*
   * @brief drive with old acceleration gas:0.00~99.99 unit:%
   * 用旧的加速度来驱动  油门范围：0.00~99.99%
   */
  virtual void Throttle(double throttle) = 0;

  /*
   * @brief drive with new acceleration/deceleration:-7.0~7.0, unit:m/s^2,
   * 以新的加速度/减速度驱动   范围：-7.0~7.0 m/s^2
   * acc:-7.0~7.0, unit:m/s^2
   */
  virtual void Acceleration(double acc) = 0;

  /*
   * @brief steering with old angle speed angle:-99.99~0.00~99.99, unit:%,
   * 以旧的角速度转向  范围：-99.99~0.00~99.99%   左正右负
   * left:+, right:-
   */
  virtual void Steer(double angle) = 0;

  /*
   * @brief steering with new angle speed angle:-99.99~0.00~99.99, unit:%,
   * 以新的角速度转向   角度angle：-99.99~0.00~99.99%   角速度angle_spd:0.00~99.99 度/秒    左正右负  
   * left:+, right:- angle_spd:0.00~99.99, unit:deg/s
   */
  virtual void Steer(double angle, double angle_spd) = 0;

  /*
   * @brief set Electrical Park Brake  设置电子驻车制动器
   */
  virtual void SetEpbBreak(const control::ControlCommand &command) = 0; //电子驻车
  virtual void SetBeam(const control::ControlCommand &command) = 0; //远近光
  virtual void SetHorn(const control::ControlCommand &command) = 0; //喇叭
  virtual void SetTurningSignal(const control::ControlCommand &command) = 0; //转向灯

  virtual void SetLimits() {} //限制

 protected:
  virtual Chassis::DrivingMode driving_mode(); //驾驶模式
  virtual void set_driving_mode(const Chassis::DrivingMode &driving_mode); //设置驾驶模式

 protected:
  canbus::VehicleParameter params_; //canbus::车辆参数类
  common::VehicleParam vehicle_params_; //common::车辆参数类
  CanSender<ChassisDetail> *can_sender_ = nullptr; //can发送者，默认空指针
  MessageManager<ChassisDetail> *message_manager_ = nullptr; //消息管理，默认空指针
  bool is_initialized_ = false;  // own by derviative concrete controller  由衍生的具体控制器管理（是否初始化）
  Chassis::DrivingMode driving_mode_ = Chassis::COMPLETE_MANUAL; //驾驶模式设置为完全人工驾驶
  bool is_reset_ = false;  // reset command from control command 重置命令来自控制命令（是否重置）
  std::mutex mode_mutex_;  // only use in this base class 模式互斥（只能用在基类）
};

}  // namespace canbus
}  // namespace apollo
