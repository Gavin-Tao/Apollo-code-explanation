/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/zhongyun/protocol/steering_control_a2.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

using ::apollo::drivers::canbus::Byte;

const int32_t Steeringcontrola2::ID = 0xA2;

// public
//构造函数
Steeringcontrola2::Steeringcontrola2() { Reset(); }

//获得周期
uint32_t Steeringcontrola2::GetPeriod() const {
  // TODO(ChaoM) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

//更新
void Steeringcontrola2::UpdateData(uint8_t* data) {
  set_p_steering_target(data, steering_target_);
  set_p_steering_enable_control(data, steering_enable_control_);
}

//重置（转向目标是0，手动控制）
void Steeringcontrola2::Reset() {
  // TODO(ChaoM) :  you should check this manually
  steering_target_ = 0.0;
  steering_enable_control_ =
      Steering_control_a2::STEERING_ENABLE_CONTROL_STEERING_MANUALCONTROL;
}

//设置转向目标，并返回指向转向目标的指针
Steeringcontrola2* Steeringcontrola2::set_steering_target(
    double steering_target) {
  steering_target_ = steering_target;
  return this;
}

// config detail: {'name': 'Steering_target', 'offset': -1638.35, 'precision':
// 0.05, 'len': 16, 'is_signed_var': False, 'physical_range': '[-32|32]', 'bit':
// 8, 'type': 'double', 'order': 'intel', 'physical_unit': 'deg'}
//偏移：-1638.35  精度：0.05   长度：16  范围：-32°~32°  位：8   类型：double   单位：度
void Steeringcontrola2::set_p_steering_target(uint8_t* data,
                                              double steering_target) {
  steering_target = ProtocolData::BoundedValue(-32.0, 32.0, steering_target);
  int x = static_cast<int>((steering_target - -1638.350000) / 0.050000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 1);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 2);
  to_set1.set_value(t, 0, 8);
}

//设定启用转向控制，并返回指向启用转向控制的指针
Steeringcontrola2* Steeringcontrola2::set_steering_enable_control(
    Steering_control_a2::Steering_enable_controlType steering_enable_control) {
  steering_enable_control_ = steering_enable_control;
  return this;
}

// config detail: {'name': 'Steering_Enable_control', 'enum': {0:
// 'STEERING_ENABLE_CONTROL_STEERING_MANUALCONTROL', 1:
// 'STEERING_ENABLE_CONTROL_STEERING_AUTOCONTROL'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0,
// 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
//enum:0~1分别为手动、自动   精度：1.0   长度：8  范围：0~1  位：0  类型：enum   单位：NONE
void Steeringcontrola2::set_p_steering_enable_control(
    uint8_t* data,
    Steering_control_a2::Steering_enable_controlType steering_enable_control) {
  int x = steering_enable_control;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
