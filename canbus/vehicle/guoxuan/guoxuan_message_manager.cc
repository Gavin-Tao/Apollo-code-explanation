//添加国轩消息管理源文件

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

#include "modules/canbus/vehicle/guoxuan/guoxuan_message_manager.h"  //在此添加国轩的消息管理器头文件

#include "modules/canbus/vehicle/guoxuan/protocol/gear_control_g1.h" //在此添加国轩换挡控制g1头文件
#include "modules/canbus/vehicle/guoxuan/protocol/steering_control_g2.h" //在此添加国轩转向控制g2
#include "modules/canbus/vehicle/guoxuan/protocol/brake_control_g4.h" //在此添加国轩制动控制g4头文件
#include "modules/canbus/vehicle/guoxuan/protocol/parking_control_g5.h" //在此添加国轩停车控制g5头文件
#include "modules/canbus/vehicle/zhongyun/protocol/torque_control_g3.h" //在此添加国轩驱动力矩控制g3头文件

//#include "modules/canbus/vehicle/zhongyun/protocol/enable_state_feedback_c3.h"
//#include "modules/canbus/vehicle/zhongyun/protocol/error_state_e1.h"
//#include "modules/canbus/vehicle/zhongyun/protocol/vehicle_state_feedback_2_c4.h"
//#include "modules/canbus/vehicle/zhongyun/protocol/vehicle_state_feedback_c1.h"

namespace apollo {
namespace canbus {
namespace guoxuan { //国轩命名空间

//国轩消息管理器构造函数
GuoxuanMessageManager::GuoxuanMessageManager() {
  // Control Messages
  AddSendProtocolData<Brakecontrolg4, true>();
  AddSendProtocolData<Gearcontrolg1, true>();
  AddSendProtocolData<Parkingcontrolg5, true>();
  AddSendProtocolData<Steeringcontrolg2, true>();
  AddSendProtocolData<Torquecontrolg3, true>();

  // Report Messages
  //AddRecvProtocolData<Enablestatefeedbackc3, true>();
  //AddRecvProtocolData<Errorstatee1, true>();
  //AddRecvProtocolData<Vehiclestatefeedback2c4, true>();
  //AddRecvProtocolData<Vehiclestatefeedbackc1, true>();
}

//国轩消息管理器析构函数
GuoxuanMessageManager::~GuoxuanMessageManager() {}

}  // namespace guoxuan
}  // namespace canbus
}  // namespace apollo
