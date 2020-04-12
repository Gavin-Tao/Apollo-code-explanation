//添加国轩车辆工厂源文件

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

#include "modules/canbus/vehicle/guoxuan/guoxuan_vehicle_factory.h"  //在此添加国轩车辆工厂头文件

#include "cyber/common/log.h"
#include "modules/canbus/vehicle/guoxuan/guoxuan_controller.h" //在此添加国轩控制器头文件
#include "modules/canbus/vehicle/guoxuan/guoxuan_message_manager.h" //在此添加国轩信息管理器头文件
#include "modules/common/util/util.h"

namespace apollo {
namespace canbus {

std::unique_ptr<VehicleController>
//在此创造国轩车辆控制器
GuoxuanVehicleFactory::CreateVehicleController() {
  return std::unique_ptr<VehicleController>(new guoxuan::GuoxuanController()); //在此返回指向创造的国轩控制器指针
}

std::unique_ptr<MessageManager<::apollo::canbus::ChassisDetail>>
//在此创造国轩消息管理器
GuoxuanVehicleFactory::CreateMessageManager() {
  return std::unique_ptr<MessageManager<::apollo::canbus::ChassisDetail>>(
      new guoxuan::GuoxuanMessageManager()); //在此返回指向创造的国轩消息管理器指针
}

}  // namespace canbus
}  // namespace apollo
