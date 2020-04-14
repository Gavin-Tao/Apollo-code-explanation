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

#include "modules/canbus/vehicle/zhongyun/protocol/brake_control_a4.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

using ::apollo::drivers::canbus::Byte;

const int32_t Brakecontrola4::ID = 0xA4; //十六进制A4的ID

// public
Brakecontrola4::Brakecontrola4() { Reset(); } //构造函数

//得到周期
uint32_t Brakecontrola4::GetPeriod() const {
  // TODO(ChaoM) :  modify every protocol's period manually  手动修改每个协议的周期
  static const uint32_t PERIOD = 20 * 1000;  //一秒等于1000毫秒
  return PERIOD;
}

//data是指向数据的指针,brake_torque_制动力矩
void Brakecontrola4::UpdateData(uint8_t* data) {
  set_p_brake_torque(data, brake_torque_);
  set_p_brake_enable_control(data, brake_enable_control_);
}

void Brakecontrola4::Reset() {
  // TODO(ChaoM) :  you should check this manually
  brake_torque_ = 0.0; //制动力矩重置为0
  brake_enable_control_ = Brake_control_a4::BRAKE_ENABLE_CONTROL_BRAKE_MANUAL; //制动启用控制模式重置为手动模式
}

//设定制动力矩，返回指向制动力矩的指针
Brakecontrola4* Brakecontrola4::set_brake_torque(double brake_torque) {
  brake_torque_ = brake_torque; //制动力矩赋值
  return this;
}

// config detail: {'name': 'brake_torque', 'offset': 0.0, 'precision': 0.05,
// 'len': 16, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 8,
// 'type': 'double', 'order': 'intel', 'physical_unit': '%'}
//偏移：0.0   精度：0.05   长度：16  是否是定义的变量：否   范围：0~100%
//位：8   类型：double   序列：intel  单位：%
//设置停车制动力矩
void Brakecontrola4::set_p_brake_torque(uint8_t* data, double brake_torque) {
  brake_torque = ProtocolData::BoundedValue(0.0, 100.0, brake_torque); //检查制动力矩是否在0.0~100.0之间，如果在就返回制动力矩。
                                                                      //否则返回边界（制动力矩比0.0小返回0.0，制动力矩比100.0大返回100.0）
  int x = static_cast<int>(brake_torque / 0.050000);
  uint8_t t = 0;

  //发来8n长度的制动力矩二进制，经过位与运算后只剩下8位有效，因为除了低八位外的高位与0取与为0   因为255除了低八位外的高位为0
  //t是二进制 t=高位0 x本身的二进制
  t = static_cast<uint8_t>(x & 0xFF); //&位与运算 0xFF=15*16+15=255  x与255转成二进制再进行位与运算  255二进制是11111111  
  //data里后面第一个八位
  Byte to_set0(data + 1); //data是指针   给data指向的数据赋值data+1指向的数据   设指针p的类型为T，则p+i=(p的值)+i*sizeof(T)的字节数
                          //data是一字节无符号指针，data+1=data存放的数据+1*一个字节数=data存放数据+1。  因为data类型占一个字节
                          ////比如：p指向的是一个整型，p+1就是移动一个整型大小，即移动4个字节，所以p+1代表的地址比p代表的地址大4.
                          //这里data是一个字节（8位）大小，所以移动八位大小
  to_set0.set_value(t, 0, 8);  //以十六进制形式在数据的低八位把t放进去（x中的后面第一个八位）
  //>>是右移运算符。 不足以0填补
  //语法格式：需要移位的数字>>移位的次数n
  //运算规则：按二进制形式把所有数字向右移动相应的位数，低位移出（舍弃），高位的空位补0。相当于除以2的n次方
  //假设x=5，那么x的二进制为0101，x>>1表示x右移1位，即把最右边一位的1删掉，变为0010，此时版权x=2；
  //仍然设x=5，二进制0101，x>>2表示x右移2位，把最右边两位的0001去掉，变为01，此时x=1。
  //x>>=1等价于x=x>>1
  x >>= 8; //x八位八位为一部分，右移八位

  //t=高位0 x本身的二进制
  t = static_cast<uint8_t>(x & 0xFF); //0xFF=15*16+15=255  x与255转成二进制再进行位与运算  255二进制是11111111  
  //data里后面第二个八位
  Byte to_set1(data + 2);
  to_set1.set_value(t, 0, 8); //在数据的低八位把t放进去（x中的后面第二个八位）
}

Brakecontrola4* Brakecontrola4::set_brake_enable_control(
    Brake_control_a4::Brake_enable_controlType brake_enable_control) {
  brake_enable_control_ = brake_enable_control; //制动启用控制类型
  return this;
}

// config detail: {'name': 'Brake_Enable_control', 'enum':
// {0: 'BRAKE_ENABLE_CONTROL_BRAKE_MANUAL',
// 1: 'BRAKE_ENABLE_CONTROL_BRAKE_AUTO'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0,
// 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
//枚举：0代表手动制动 1代表自动制动  精度：1.0  长度：8  是否是变量：否  偏移：0.0
//范围：0~1  位：0   种类：enum   序列：intel  单位：NONE
void Brakecontrola4::set_p_brake_enable_control(
    uint8_t* data,
    Brake_control_a4::Brake_enable_controlType brake_enable_control) {
  int x = brake_enable_control; //启用制动控制

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
