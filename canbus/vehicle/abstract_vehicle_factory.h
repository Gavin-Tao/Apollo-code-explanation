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
 * @file
 */

#pragma once

#include <memory>

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"

/**
 * @namespace apollo::canbus
 * @brief apollo::canbus
 */
namespace apollo {
namespace canbus {

/**
 * @class AbstractVehicleFactory
 *
 * @brief this class is the abstract factory following the AbstractFactory
 * design pattern. It can create VehicleController and MessageManager based on
 * a given VehicleParameter.
 */
//此类是遵循AbstractFactory设计模式的抽象工厂。 它可以基于给定的VehicleParameter创建VehicleController和MessageManager。
class AbstractVehicleFactory {
 public:
  /**
   * @brief destructor析构函数
   */
  virtual ~AbstractVehicleFactory() = default;//default关键字，功能是指示编译器完成类特殊成员函数的默认生成工作

  /**
   * @brief the interface of creating a VehicleController class 创建VehicleController类的接口
   * @returns a unique pointer that points to the created VehicleController
   * object. 返回指向创建的VehicleController对象的唯一指针。
   */
  virtual std::unique_ptr<VehicleController> CreateVehicleController() = 0;

  /**
   * @brief the interface of creating a MessageManager class 创建MessageMangager类的接口
   * @returns a unique pointer that points to the created MessageManager object.
   *          返回指向创建的MessageManager对象的唯一指针
   */
  virtual std::unique_ptr<MessageManager<ChassisDetail>>
  CreateMessageManager() = 0;

  /**
   * @brief set VehicleParameter. 设置车辆参数，将传入的车辆参数类赋值给成员变量的车辆参数类
   */
  void SetVehicleParameter(const VehicleParameter &vehicle_paramter);

 private:
  VehicleParameter vehicle_parameter_; //成员变量  车辆参数
};

}  // namespace canbus
}  // namespace apollo
