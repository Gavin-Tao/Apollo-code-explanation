//添加国轩车辆工厂



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

/**
 * @file guoxuan_vehicle_factory.h
 */

#pragma once

#include <memory>

#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/canbus/vehicle/abstract_vehicle_factory.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"

/**
 * @namespace apollo::canbus
 * @brief apollo::canbus
 */
namespace apollo {
namespace canbus {

/**
 * @class GuoxuanVehicleFactory
 *
 * @brief this class is inherited from AbstractVehicleFactory. It can be used to
 * create controller and message manager for guoxuan vehicle.
 */
//定义国轩车辆工厂类（继承抽象车辆工厂）
class GuoxuanVehicleFactory : public AbstractVehicleFactory {
 public:
  /**
   * @brief destructor
   */
  //虚析构函数，不可以发生隐式转换
  virtual ~GuoxuanVehicleFactory() = default;

  /**
   * @brief create guoxuan vehicle controller
   * @returns a unique_ptr that points to the created controller
   */
  std::unique_ptr<VehicleController> CreateVehicleController() override;

  /**
   * @brief create guoxuan message manager
   * @returns a unique_ptr that points to the created message manager
   */
  std::unique_ptr<MessageManager<::apollo::canbus::ChassisDetail>>
  CreateMessageManager() override;
};

}  // namespace canbus
}  // namespace apollo
