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

#include "modules/canbus/vehicle/zhongyun/protocol/parking_control_a5.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

using ::apollo::drivers::canbus::Byte;

const int32_t Parkingcontrola5::ID = 0xA5;

// public
//构造函数
Parkingcontrola5::Parkingcontrola5() { Reset(); }

//获得周期
uint32_t Parkingcontrola5::GetPeriod() const {
  // TODO(ChaoM) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000; //1s=1000ms
  return PERIOD;
}

//更新数据
void Parkingcontrola5::UpdateData(uint8_t* data) {
  set_p_parking_target(data, parking_target_);
  set_p_parking_enable_control(data, parking_enable_control_);
}

//重置（停车目标设定为停车目标释放，手动控制）
void Parkingcontrola5::Reset() {
  // TODO(ChaoM) :  you should check this manually
  parking_target_ = Parking_control_a5::PARKING_TARGET_RELEASE; //停车目标释放（现在不需要停车了，本来的停车目标释放掉）
  parking_enable_control_ =
      Parking_control_a5::PARKING_ENABLE_CONTROL_PARKING_MANUALCONTROL; //手动控制
}

//设定停车目标，并返回指向停车目标的指针
Parkingcontrola5* Parkingcontrola5::set_parking_target(
    Parking_control_a5::Parking_targetType parking_target) {
  parking_target_ = parking_target;
  return this;
}

// config detail: {'name': 'Parking_target', 'enum': {0:
// 'PARKING_TARGET_RELEASE', 1: 'PARKING_TARGET_PARKING_TRIGGER'},
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'intel',
// 'physical_unit': ''}
//enum：0~1分别为停车目标释放，停车目标触发   精度：1.0   长度：8  范围：0~1  位：8   类型：enum   单位：NONE
void Parkingcontrola5::set_p_parking_target(
    uint8_t* data, Parking_control_a5::Parking_targetType parking_target) {
  int x = parking_target;

  Byte to_set(data + 1);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8); //以16进制赋值8位
}

//设定启用停车控制，并返回指针
Parkingcontrola5* Parkingcontrola5::set_parking_enable_control(
    Parking_control_a5::Parking_enable_controlType parking_enable_control) {
  parking_enable_control_ = parking_enable_control;
  return this;
}

// config detail: {'name': 'Parking_Enable_control', 'enum': {0:
// 'PARKING_ENABLE_CONTROL_PARKING_MANUALCONTROL', 1:
// 'PARKING_ENABLE_CONTROL_PARKING_AUTOCONTROL'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0,
// 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Parkingcontrola5::set_p_parking_enable_control(
    uint8_t* data,
    Parking_control_a5::Parking_enable_controlType parking_enable_control) {
  int x = parking_enable_control;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8); //以16进制赋值8位
}

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
