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

#include "modules/canbus/vehicle/abstract_vehicle_factory.h"
#include "modules/common/util/factory.h"

/**
 * @namespace apollo::canbus
 * @brief apollo::canbus
 */
namespace apollo {
namespace canbus {

/**
 * @class VehicleFactory
 *
 * @brief This class is a factory class that will generate different
 * vehicle factories based on the vehicle brand.
 * 该类是工厂类，它将根据汽车品牌产生不同的汽车工厂。
 */
/*template <typename IdentifierType, class AbstractProduct,
          class ProductCreator = AbstractProduct *(*)(),
          class MapContainer = std::map<IdentifierType, ProductCreator>>
*/
//VehicleBrand车辆品牌，相当于模板中的IdentifierType标识符类型
//AbstractVehicleFactory抽象车辆工厂，相当于模板中的AbstractProduct抽象产品
class VehicleFactory
    : public common::util::Factory<apollo::common::VehicleBrand,
                                   AbstractVehicleFactory> {
 public:
  /**
   * @brief register supported vehicle factories.
   * 注册支持的车辆工厂。
   */
  void RegisterVehicleFactory(); //注册车辆工厂

  /**
   * @brief Creates an AbstractVehicleFactory object based on vehicle_parameter
   * 根据vehicle_parameter创建一个AbstractVehicleFactory对象
   * @param vehicle_parameter is defined in vehicle_parameter.proto  
   * 车辆参数,定义在vehicle_parameter.proto（品牌，最大发动机塔板数，最大失败尝试次数，驾驶模式）
   */
  std::unique_ptr<AbstractVehicleFactory> CreateVehicle(
      const VehicleParameter &vehicle_parameter);
};

}  // namespace canbus
}  // namespace apollo
