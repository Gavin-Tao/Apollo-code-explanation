//看不懂的话，先看435-448的注释
//添加国轩控制器的源文件

/* Copyright 2019 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "modules/canbus/vehicle/guoxuan/guoxuan_controller.h"

#include "modules/common/proto/vehicle_signal.pb.h"

#include "cyber/common/log.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/canbus/vehicle/guoxuan/guoxuan_message_manager.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace guoxuan {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {

const int32_t kMaxFailAttempt = 10; //最大错误尝试次数10
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1; //检查转向单元响应的gflag
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2; //检查速度单元响应的gflag
}  // namespace

//控制器初始化，返回故障码
ErrorCode GuoxuanController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::ChassisDetail>* const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail>* const message_manager) {
  //如果已经被初始化则为真，输出已经被初始化并返回canbus的故障码
  if (is_initialized_) {
    AINFO << "GuoxuanController has already been initialized.";
    return ErrorCode::CANBUS_ERROR;
  }
  //没被初始化，往下执行
  vehicle_params_.CopyFrom(
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param()); //给common::车辆参数赋值，复制而来
  params_.CopyFrom(params); //给canbus::成员变量车辆参数赋值
  //如果canbus::车辆参数没有驾驶模式，则！params_has_driving_mode为真，输出车辆配置文件没有设置驾驶模式并返回canbus故障码
  if (!params_.has_driving_mode()) {
    AERROR << "Vehicle conf pb not set driving_mode.";
    return ErrorCode::CANBUS_ERROR;
  }

  //如果为空指针返回故障码，否则给成员变量can_sender_赋值
  if (can_sender == nullptr) {
    AERROR << "Protocol can sender is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  can_sender_ = can_sender; 

  //如果为空指针返回故障码，否则给成员变量message_manager_赋值
  if (message_manager == nullptr) {
    AERROR << "Protocol manager is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  message_manager_ = message_manager;

  // Sender part  发送部分
  //制动
  brake_control_g4_ = dynamic_cast<Brakecontrolg4*>(
      message_manager_->GetMutableProtocolDataById(Brakecontrolg4::ID)); //获得指向动态协议数据（制动控制）的指针
  //如果制动控制指针为空，则返回故障信息和故障码
  if (brake_control_g4_ == nullptr) {
    AERROR << "Brakecontrolg4 does not exist in the GuoxuanMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  //换挡
  gear_control_g1_ = dynamic_cast<Gearcontrolg1*>(
      message_manager_->GetMutableProtocolDataById(Gearcontrolg1::ID)); //获得指向动态协议数据（换挡控制）的指针
  //如果换挡控制指针为空，则返回故障信息和故障码
  if (gear_control_g1_ == nullptr) {
    AERROR << "Gearcontrolg1 does not exist in the GuoxuanMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  //停车
  parking_control_g5_ = dynamic_cast<Parkingcontrolg5*>(
      message_manager_->GetMutableProtocolDataById(Parkingcontrolg5::ID)); //获得指向动态协议数据（停车控制）的指针
  //如果停车控制指针为空，则返回故障信息和故障码
  if (parking_control_g5_ == nullptr) {
    AERROR << "Parkingcontrolg5 does not exist in the GuoxuanMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  //转向
  steering_control_g2_ = dynamic_cast<Steeringcontrolg2*>(
      message_manager_->GetMutableProtocolDataById(Steeringcontrolg2::ID)); //获得指向动态协议数据（转向控制）的指针
  //如果转向控制指针为空，则返回故障信息和故障码
  if (steering_control_g2_ == nullptr) {
    AERROR << "Steeringcontrolg2 does not exist in the GuoxuanMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  //力矩
  torque_control_g3_ = dynamic_cast<Torquecontrolg3*>(
      message_manager_->GetMutableProtocolDataById(Torquecontrolg3::ID)); //获得指向动态协议数据（力矩控制）的指针
  //如果力矩控制指针为空，则返回故障信息和故障码
  if (torque_control_g3_ == nullptr) {
    AERROR << "Torquecontrolg3 does not exist in the GuoxuanMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  //在can_sender_中添加消息（控制命令id，指向动态协议数据的指针）
  //如果为true，则将协议数据中的所有位初始化为1。 默认情况下为false。
  can_sender_->AddMessage(Brakecontrolg4::ID, brake_control_g4_, false);
  can_sender_->AddMessage(Gearcontrolg1::ID, gear_control_g1_, false);
  can_sender_->AddMessage(Parkingcontrolg5::ID, parking_control_g5_, false);
  can_sender_->AddMessage(Steeringcontrolg2::ID, steering_control_g2_, false);
  can_sender_->AddMessage(Torquecontrolg3::ID, torque_control_g3_, false);

  // Need to sleep to ensure all messages received
  //需要休眠以确保收到所有消息
  AINFO << "GuoxuanController is initialized."; //输出控制器已经被初始化。

  is_initialized_ = true;  //将默认为false的is_initialized_赋值为true
  return ErrorCode::OK; //返回OK故障码，代表没有故障
}

//析构函数
GuoxuanController::~GuoxuanController() {}

//打开控制器
bool GuoxuanController::Start() {
  //检查是否初始化完成
  if (!is_initialized_) {
    AERROR << "GuoxuanController has NOT been initialized.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); }; //安全线程，检查与底盘通讯情况
  thread_.reset(new std::thread(update_func)); //指向线程的指针更新重置

  return true;
}

//停止控制器
void GuoxuanController::Stop() {
  //如果没有初始化为真，则控制器停止或者不正确启动
  if (!is_initialized_) {
    AERROR << "GuoxuanController stops or starts improperly!";
    return;
  }

  //如果初始化了，往下执行
  //如果指向线程的指针不为空且可以加入线程         joinable()   如果可以加入该线程，则返回true
  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();  //将控制器的线程加入线程
    thread_.reset();  //控制器线程线程重置
    AINFO << "GuoxuanController stopped.";  //控制器已经停止
  }
}

//底盘
Chassis GuoxuanController::chassis() {
  chassis_.Clear(); //底盘清理（重置）

  ChassisDetail chassis_detail;  //底盘信息
  message_manager_->GetSensorData(&chassis_detail); //消息管理器获得传感器数据（指向底盘信息的指针）

  // 1, 2
  //设置驾驶模式，设置故障码
  if (driving_mode() == Chassis::EMERGENCY_MODE) {
    set_chassis_error_code(Chassis::NO_ERROR);
  }

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3
  //设置引擎启动
  chassis_.set_engine_started(true);
  // if there is not zhongyun, no chassis detail can be retrieved and return
  //如果没有中云，则无法检索和返回底盘详细信息
  if (!chassis_detail.has_guoxuan()) {
    AERROR << "NO GUOXUAN chassis information!";
    return chassis_;
  }
  Guoxuan gx = chassis_detail.guoxuan();  //中云的底盘信息

  // 4 engine_rpm
  //引擎转速 转/分
  //如果底盘信息有车辆状态反馈信息2和有电机转速信息时为真，则设置底盘信息里的引擎转速（电机转速）
  if (gx.has_vehicle_state_feedback_2_g4() &&
      gx.vehicle_state_feedback_2_g4().has_motor_speed()) {
    chassis_.set_engine_rpm(
        static_cast<float>(zhy.vehicle_state_feedback_2_g4().motor_speed()));
  } else {
    chassis_.set_engine_rpm(0);  //否则设置引擎转速为0
  }
  // 5 speed_mps
  //车速  m/s
  //如果底盘信息有车辆状态反馈信息和有车速信息时为真，则设置底盘信息里的车速
  if (gx.has_vehicle_state_feedback_g1() &&
      gx.vehicle_state_feedback_g1().has_speed()) {
    chassis_.set_speed_mps(
        static_cast<float>(gx.vehicle_state_feedback_g1().speed()));
  } else {
    chassis_.set_speed_mps(0);  //否则设置车速为0
  }
  // 6
  //设置燃料范围0
  chassis_.set_fuel_range_m(0);

  // 7 acc_pedal
  //油门踏板
  //如果中云底盘信息有车辆状态反馈信息2和有驱动力矩返回信息时为真，则设置节气门开度百分比
  if (gx.has_vehicle_state_feedback_2_g4() &&
      gx.vehicle_state_feedback_2_g4().has_driven_torque_feedback()) {
    chassis_.set_throttle_percentage(static_cast<float>(
        gx.vehicle_state_feedback_2_g4().driven_torque_feedback()));
  } else {
    chassis_.set_throttle_percentage(0);  //否则设置节气门开度为0
  }
  // 8 brake_pedal
  //制动踏板
  //如果中云底盘信息有车辆状态反馈信息和制动力矩返回信息时为真，则设置制动开度百分比
  if (gx.has_vehicle_state_feedback_g1() &&
      gx.vehicle_state_feedback_g1().has_brake_torque_feedback()) {
    chassis_.set_brake_percentage(static_cast<float>(
        gx.vehicle_state_feedback_g1().brake_torque_feedback()));
  } else {
    chassis_.set_brake_percentage(0); //否则设置制动开度为0
  }
  // 9 gear position
  //档位
  //如果中云底盘信息有车辆状态反馈信息和实际档位状态信息时为真，则设置底盘的档位信息
  if (gx.has_vehicle_state_feedback_g1() &&
      gx.vehicle_state_feedback_g1().has_gear_state_actual()) {
    switch (gx.vehicle_state_feedback_g1().gear_state_actual()) {
      case Vehicle_state_feedback_g1::GEAR_STATE_ACTUAL_D: {
        chassis_.set_gear_location(Chassis::GEAR_DRIVE); //实际档位是前进挡
      } break;
      case Vehicle_state_feedback_g1::GEAR_STATE_ACTUAL_N: {
        chassis_.set_gear_location(Chassis::GEAR_NEUTRAL); //实际档位是空挡
      } break;
      case Vehicle_state_feedback_g1::GEAR_STATE_ACTUAL_R: {
        chassis_.set_gear_location(Chassis::GEAR_REVERSE); //实际档位是倒挡
      } break;
      case Vehicle_state_feedback_g1::GEAR_STATE_ACTUAL_P: {
        chassis_.set_gear_location(Chassis::GEAR_PARKING); //实际档位是停车挡
      } break;
      default:
        chassis_.set_gear_location(Chassis::GEAR_INVALID); //实际档位是无效挡
        break;
    }
  } else {
    chassis_.set_gear_location(Chassis::GEAR_NONE); //否则设置底盘档位信息为NONE没有
  }
  // 11 steering_percentage
  //转向百分比
  //如果中云底盘信息有车辆状态反馈信息和有实际转向信息时为真，设置转向百分比
  if (gx.has_vehicle_state_feedback_g1() &&
      gx.vehicle_state_feedback_g1().has_steering_actual()) {
    chassis_.set_steering_percentage(static_cast<float>(
        gx.vehicle_state_feedback_g1().steering_actual() * 100.0 /
        vehicle_params_.max_steer_angle() * M_PI / 180)); //设置转向百分比
  } else {
    chassis_.set_steering_percentage(0);  //否则设置转向百分比为0
  }
  // 12 epb
  //电子驻车制动
  //如果中云底盘信息有车辆状态反馈信息和有实际停车信息时为真，设置底盘是否可以停车制动信息
  if (gx.has_vehicle_state_feedback_g1() &&
      gx.vehicle_state_feedback_g1().has_parking_actual()) {
    chassis_.set_parking_brake(
        gx.vehicle_state_feedback_g1().parking_actual() ==
        Vehicle_state_feedback_g1::PARKING_ACTUAL_PARKING_TRIGGER); //设置底盘是否停车制动信息（根据实际停车信息是否可以触发）
        //Vehicle_state_feedback_g1为定义的消息类型，PARKING_ACTUAL_PARKING_TRIGGER为该消息类型下的枚举
  } else {
    chassis_.set_parking_brake(false); //否则设置不可以停车制动
  }
  // 13 error mask
  //错误掩码，默认底盘错误掩码为0
  //如果有错误掩码则为真，设置底盘的错误掩码信息
  if (chassis_error_mask_) {
    chassis_.set_chassis_error_mask(chassis_error_mask_);  //设置底盘的错误掩码信息
  }
  // Give engage_advice based on error_code and canbus feedback
  //根据error_code和canbus反馈提供engage_advice参与意见
  //chassis_.parking_brake()是bool类型，如果在电子驻车制动则为true，则!chassis_.parking_brake()为false，则if判断语句为假，则不可以参与意见
  if (!chassis_error_mask_ && !chassis_.parking_brake()) {
    chassis_.mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::READY_TO_ENGAGE); //参与意见
  } else {
    chassis_.mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE); //不可以参与意见
    chassis_.mutable_engage_advice()->set_reason(
        "CANBUS not ready, epb is not released or firmware error!"); //理由：CANBUS未准备好，epb未释放或固件错误！
  }
  return chassis_; //返回底盘
}

//紧急
void GuoxuanController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);  //设定紧急驾驶模式
  ResetProtocol(); //重置协议
}

//是否为自动驾驶模式，返回故障码
ErrorCode GuoxuanController::EnableAutoMode() {
  //如果驾驶模式时自动驾驶模式，输出已经自动驾驶模式并返回OK故障码
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "Already in COMPLETE_AUTO_DRIVE mode.";
    return ErrorCode::OK;
  }
  //对指向转向控制的指针设定转向可以控制（启用转向自动控制）
  steering_control_g2_->set_steering_enable_control(
      Steering_control_g2::STEERING_ENABLE_CONTROL_STEERING_AUTOCONTROL);
  //换挡（启用换挡自动控制）
  gear_control_g1_->set_gear_enable_control(
      Gear_control_g1::GEAR_ENABLE_CONTROL_GEAR_AUTOCONTROL);
  //力矩控制（启用驱动自动控制）
  torque_control_g3_->set_driven_enable_control(
      Torque_control_g3::DRIVEN_ENABLE_CONTROL_DRIVE_AUTO);
  //制动控制（启用制动自动控制）
  brake_control_g4_->set_brake_enable_control(
      Brake_control_g4::BRAKE_ENABLE_CONTROL_BRAKE_AUTO);
  //停车控制（启用停车自动控制）
  parking_control_g5_->set_parking_enable_control(
      Parking_control_g5::PARKING_ENABLE_CONTROL_PARKING_AUTOCONTROL);

  can_sender_->Update(); //can发送者消息更新
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG; //检查转向单元响应和速度单元响应
  //如果没有响应则条件语句成立，输出未能切换到COMPLETE_AUTO_DRIVE模式并进入紧急、设置底盘故障码并返回故障码
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  //如果有响应，则设定驾驶模式为自动驾驶模式并输出切换到自动驾驶模式并返回OK
  set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
  return ErrorCode::OK;
}

//返回不能够自动驾驶模式的故障码
ErrorCode GuoxuanController::DisableAutoMode() {
  ResetProtocol(); //重置协议
  can_sender_->Update(); //can发送者更新消息
  set_driving_mode(Chassis::COMPLETE_MANUAL); //设置完全手工驾驶模式
  set_chassis_error_code(Chassis::NO_ERROR); //设置底盘故障码为NO_ERROR
  AINFO << "Switch to COMPLETE_MANUAL mode."; //输出切换到完全手工驾驶
  return ErrorCode::OK; //返回OK
}

//返回只能转向模式的故障码
ErrorCode GuoxuanController::EnableSteeringOnlyMode() {
  //如果现在的驾驶模式是自动驾驶或者仅转向模式为真
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY); //设定仅转向自动模式
    AINFO << "Already in AUTO_STEER_ONLY mode"; //输出早已经是仅转向模式
    return ErrorCode::OK; //返回OK
  }
  //否则对转向、换挡、前进、制动设置启用的控制模式,除了设置转向是自动控制模式外，其余均设置为手工控制
  steering_control_g2_->set_steering_enable_control(
      Steering_control_g2::STEERING_ENABLE_CONTROL_STEERING_AUTOCONTROL); //设定启用转向控制模式为转向自动控制
  gear_control_g1_->set_gear_enable_control(
      Gear_control_g1::GEAR_ENABLE_CONTROL_GEAR_MANUALCONTROL); //设定启用换挡控制模式为手工换挡控制
  torque_control_g3_->set_driven_enable_control(
      Torque_control_g3::DRIVEN_ENABLE_CONTROL_DRIVE_MANUAL); //驱动力矩控制模式为手工驱动控制
  brake_control_g4_->set_brake_enable_control(
      Brake_control_g4::BRAKE_ENABLE_CONTROL_BRAKE_MANUAL); //手工制动控制
  parking_control_g5_->set_parking_enable_control(
      Parking_control_g5::PARKING_ENABLE_CONTROL_PARKING_MANUALCONTROL); //手工停车控制

  can_sender_->Update(); //can发送者更新消息
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG; //检查转向单元和速度单元的响应
  //如果没有响应，则if判断语句为真
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode."; //失败切换到自动驾驶模式
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR); //返回故障码
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::AUTO_STEER_ONLY);  //设定仅转向模式
  AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok."; //切换到自动驾驶模式
  return ErrorCode::OK; //返回OK
}

//返回仅速度模式的故障码
ErrorCode GuoxuanController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  //换挡、驱动力矩、制动、停车设置为自动模式，转向设置为手动模式
  steering_control_g2_->set_steering_enable_control(
      Steering_control_g2::STEERING_ENABLE_CONTROL_STEERING_MANUALCONTROL);
  gear_control_g1_->set_gear_enable_control(
      Gear_control_g1::GEAR_ENABLE_CONTROL_GEAR_AUTOCONTROL);
  torque_control_g3_->set_driven_enable_control(
      Torque_control_g3::DRIVEN_ENABLE_CONTROL_DRIVE_AUTO);
  brake_control_g4_->set_brake_enable_control(
      Brake_control_g4::BRAKE_ENABLE_CONTROL_BRAKE_AUTO);
  parking_control_g5_->set_parking_enable_control(
      Parking_control_g5::PARKING_ENABLE_CONTROL_PARKING_AUTOCONTROL);

  can_sender_->Update();
  if (CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, true) == false) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode."; 
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR); //返回故障码
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::AUTO_SPEED_ONLY);
  AINFO << "Switch to AUTO_SPEED_ONLY mode ok.";
  return ErrorCode::OK;
}

// NEUTRAL, REVERSE, DRIVE, PARK
//Chassis对应的定义的消息类型，GearPosition对应枚举类型，gear_position对应optional GearPosition gear_location = 23;重命名的名字，这里是gear_position
void GuoxuanController::Gear(Chassis::GearPosition gear_position) {
  //如果驾驶模式不是自动驾驶也不是仅速度控制，那么输出不需要设置换挡，并返回OK
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "This drive mode no need to set gear.";
    return;
  }

  //档位非NONE时，执行case语句
  switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {  //Chassis对应定义的消息类型，GEAR_NEUTRAL对应GearPosition里的各种可能的值
      gear_control_g1_->set_gear_state_target(   //gear_control_a1是指针，对应gear_control_a1.cc，set_gear_state_target是文件里的函数
          Gear_control_g1::GEAR_STATE_TARGET_N);  //Gear_control_a1对应定义的消息类型，GEAR_STATE_TARGET_N对应Gear_state_targetType中的各种可能值
      break;
    }
    case Chassis::GEAR_REVERSE: {
      gear_control_g1_->set_gear_state_target(
          Gear_control_g1::GEAR_STATE_TARGET_R);
      break;
    }
    case Chassis::GEAR_DRIVE: {
      gear_control_g1_->set_gear_state_target(
          Gear_control_g1::GEAR_STATE_TARGET_D);
      break;
    }
    case Chassis::GEAR_PARKING: {
      gear_control_g1_->set_gear_state_target(
          Gear_control_g1::GEAR_STATE_TARGET_P);
      break;
    }

    //无效的档位
    case Chassis::GEAR_INVALID: {
      AERROR << "Gear command is invalid!";
      gear_control_g1_->set_gear_state_target(
          Gear_control_g1::GEAR_STATE_TARGET_INVALID);
      break;
    }
    //其他情况都设置为停车挡
    default: {
      gear_control_g1_->set_gear_state_target(
          Gear_control_g1::GEAR_STATE_TARGET_P);
      break;
    }
  }
}

// brake with brake pedal
// pedal:0.00~99.99, unit:percentage
void GuoxuanController::Brake(double pedal) {
  //如果驾驶模式不是自动驾驶模式也不是仅速度模式为真，输出不需要再设置制动踏板。
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set brake pedal.";
    return;
  }
  brake_control_g4_->set_brake_torque(pedal);  //返回指向制动踏板的指针
}

// drive with throttle pedal
// pedal:0.00~99.99 unit:percentage
void GuoxuanController::Throttle(double pedal) {
  //如果驾驶模式不是自动驾驶模式也不是仅速度模式为真，输出不需要再设置油门踏板
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set throttle pedal.";
    return;
  }
  torque_control_g3_->set_driven_torque(pedal); //返回指向油门踏板的指针
}

// confirm the car is driven by acceleration command or throttle/brake pedal
// drive with acceleration/deceleration  加减速度
// acc:-7.0 ~ 5.0, unit:m/s^2
void GuoxuanController::Acceleration(double acc) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration."; //不需要设置加减速度
    return;
  }
  // None   需要确认车是通过加减速度驱动还是通过油门/制动踏板驱动
}

// guoxuan default, -30 ~ 30, left:+, right:- 中云默认-30%~30%，左正右负
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:%, left:-, right:+
//需要与控制模块兼容，因此以旧角速度反向转向
//角度：-99.99〜0.00〜99.99，单位:%，左：-，右：+
void GuoxuanController::Steer(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0; //真实角度百分比
  steering_control_g2_->set_steering_target(real_angle); //设定转向目标
}

// steering with new angle speed   转向以新的角速度
// guoxuan has no angle_speed
// angle:-30~30, unit:%, left:+, right:-
void GuoxuanController::Steer(double angle, double angle_spd) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0; //真实角度百分比
  steering_control_g2_->set_steering_target(real_angle); //设定转向目标
}

//电子驻车制动
void GuoxuanController::SetEpbBreak(const ControlCommand& command) {
  //指令中是否有停车制动
  if (command.parking_brake()) {
    parking_control_g5_->set_parking_target(
        Parking_control_g5::PARKING_TARGET_PARKING_TRIGGER); //触发电子驻车制动
  } else {
    parking_control_g5_->set_parking_target(
        Parking_control_g5::PARKING_TARGET_RELEASE); //释放电子驻车制动
  }
}

//远近光灯
void GuoxuanController::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    // None
  } else if (command.signal().low_beam()) {
    // None
  } else {
    // None
  }
}

//喇叭
void GuoxuanController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    // None
  } else {
    // None
  }
}

//信号灯
void GuoxuanController::SetTurningSignal(const ControlCommand& command) {
  // Set Turn Signal
  // None
}

//重置协议（发送过来的数据）
void GuoxuanController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool GuoxuanController::CheckChassisError() {
  ChassisDetail chassis_detail;//底盘信息
  message_manager_->GetSensorData(&chassis_detail); //获取底盘信息
  //检查底盘信息是否有中云信息
  if (!chassis_detail.has_guoxuan()) {
    AERROR_EVERY(100) << "ChassisDetail has NO guoxuan vehicle info."
                      << chassis_detail.DebugString();
    return false;
  }
  Guoxuan gx = chassis_detail.guoxuan();
  // check steer error
  //检查转向错误
  if (gx.has_error_state_g1() &&
      gx.error_state_g1().has_steering_error_code()) {
    if (gx.error_state_g1().steering_error_code() ==
        Error_state_g1::STEERING_ERROR_CODE_ERROR) {
      return true;
    }
  }
  // check ems error
  //检查驱动错误
  if (gx.has_error_state_g1() &&
      gx.error_state_g1().has_driven_error_code()) {
    if (gx.error_state_g1().driven_error_code() ==
        Error_state_g1::DRIVEN_ERROR_CODE_ERROR) {
      return true;
    }
  }
  // check eps error
  //检查制动错误
  if (gx.has_error_state_g1() && gx.error_state_g1().has_brake_error_code()) {
    if (gx.error_state_g1().brake_error_code() ==
        Error_state_g1::BRAKE_ERROR_CODE_ERROR) {
      return true;
    }
  }
  // check gear error
  //检查换挡错误
  if (gx.has_error_state_g1() && gx.error_state_g1().has_gear_error_msg()) {
    if (gx.error_state_g1().gear_error_msg() ==
        Error_state_g1::GEAR_ERROR_MSG_ERROR) {
      return true;
    }
  }
  // check parking error
  //检查停车错误
  if (gx.has_error_state_g1() &&
      gx.error_state_g1().has_parking_error_code()) {
    if (gx.error_state_g1().parking_error_code() ==
        Error_state_g1::PARKING_ERROR_CODE_ERROR) {
      return true;
    }
  }
  return false;
}
//安全预警
void GuoxuanController::SecurityDogThreadFunc() {
  int32_t vertical_ctrl_fail = 0;
  int32_t horizontal_ctrl_fail = 0;

  if (can_sender_ == nullptr) {
    AERROR << "Failed to run SecurityDogThreadFunc() because can_sender_ is "
              "nullptr.";  //不能运行安全预警函数，因为can_sender_是空指针
    return;
  }
  //当can_sender_没有运行时，报警
  while (!can_sender_->IsRunning()) {
    std::this_thread::yield();
  }

  std::chrono::duration<double, std::micro> default_period{50000};  //报警持续时间
  int64_t start = 0;
  int64_t end = 0;
  while (can_sender_->IsRunning()) {
    start = absl::ToUnixMicros(::apollo::common::time::Clock::Now());
    const Chassis::DrivingMode mode = driving_mode(); //设定驾驶模式
    bool emergency_mode = false; //紧急模式为false

    // 1. horizontal control check
    //水平控制检查
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_STEER_ONLY) &&
        !CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false)) {
      ++horizontal_ctrl_fail;
      //水平控制失败次数大于最大失败尝试次数时为真
      if (horizontal_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true; //紧急模式为true
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION); //设定底盘故障码（手动干预）
      }
    } else {
      horizontal_ctrl_fail = 0; //否则水平控制失败次数为0
    }

    // 2. vertical control check
    //纵向控制检查
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_SPEED_ONLY) &&
        !CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, false)) {
      ++vertical_ctrl_fail;
      if (vertical_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true; //紧急模式为true
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION); //手动干预
      }
    } else {
      vertical_ctrl_fail = 0;
    }
    if (CheckChassisError()) {
      set_chassis_error_code(Chassis::CHASSIS_ERROR); //设定底盘故障码
      emergency_mode = true; //紧急模式
    }

    //紧急模式为true且驾驶模式不是紧急模式时为真
    if (emergency_mode && mode != Chassis::EMERGENCY_MODE) {
      set_driving_mode(Chassis::EMERGENCY_MODE); //设定紧急模式
      message_manager_->ResetSendMessages(); //重置发送来的数据
    }
    end = absl::ToUnixMicros(::apollo::common::time::Clock::Now());
    std::chrono::duration<double, std::micro> elapsed{end - start}; //计时
    //计时小于周期时间，休眠到周期时间结束为止
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
    } else {
      AERROR
          << "Too much time consumption in GuoxuanController looping process:" //否则输出ZhongyunController循环过程中耗时过多
          << elapsed.count();
    }
  }
}

bool GuoxuanController::CheckResponse(const int32_t flags, bool need_wait) {
  // for Guoxuan, CheckResponse commonly takes 300ms. We leave a 100ms buffer
  // for it.
  //对于Guoxuan，CheckResponse通常需要300毫秒。 我们为其保留了100ms的缓冲区。
  int32_t retry_num = 20;
  ChassisDetail chassis_detail;
  bool is_eps_online = false;
  bool is_vcu_online = false;
  bool is_esp_online = false;

  do {
    if (message_manager_->GetSensorData(&chassis_detail) != ErrorCode::OK) {
      AERROR_EVERY(100) << "get chassis detail failed.";
      return false;
    }
    bool check_ok = true;
    if (flags & CHECK_RESPONSE_STEER_UNIT_FLAG) {
      is_eps_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_eps_online() &&
                      chassis_detail.check_response().is_eps_online();
      check_ok = check_ok && is_eps_online;
    }

    if (flags & CHECK_RESPONSE_SPEED_UNIT_FLAG) {
      is_vcu_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_vcu_online() &&
                      chassis_detail.check_response().is_vcu_online();
      is_esp_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_esp_online() &&
                      chassis_detail.check_response().is_esp_online();
      check_ok = check_ok && is_vcu_online && is_esp_online;
    }
    if (need_wait) {
      --retry_num;
      std::this_thread::sleep_for(
          std::chrono::duration<double, std::milli>(20));
    }
    if (check_ok) {
      return true;
    } else {
      AINFO << "Need to check response again.";
    }
  } while (need_wait && retry_num);

  AINFO << "check_response fail: is_eps_online:" << is_eps_online
        << ", is_vcu_online:" << is_vcu_online
        << ", is_esp_online:" << is_esp_online;
  return false;
}

void GuoxuanController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t GuoxuanController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode GuoxuanController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void GuoxuanController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
