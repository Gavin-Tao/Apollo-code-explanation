//添加国轩的guoxuan_vehicle_factory.h
//注册国轩车辆工厂

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

#include "modules/canbus/vehicle/vehicle_factory.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/canbus/vehicle/ch/ch_vehicle_factory.h"
#include "modules/canbus/vehicle/ge3/ge3_vehicle_factory.h"
#include "modules/canbus/vehicle/gem/gem_vehicle_factory.h"
#include "modules/canbus/vehicle/lexus/lexus_vehicle_factory.h"
#include "modules/canbus/vehicle/lincoln/lincoln_vehicle_factory.h"
#include "modules/canbus/vehicle/transit/transit_vehicle_factory.h"
#include "modules/canbus/vehicle/wey/wey_vehicle_factory.h"
#include "modules/canbus/vehicle/zhongyun/zhongyun_vehicle_factory.h"
#include "modules/canbus/vehicle/guoxuan/guoxuan_vehicle_factory.h"   //在此添加国轩的guoxuan_vehicle_factory.h

namespace apollo {
namespace canbus {

//支持的车辆工厂
void VehicleFactory::RegisterVehicleFactory() {
  //Register(id标识符，指向注册类实例的指针)
  Register(apollo::common::LINCOLN_MKZ, []() -> AbstractVehicleFactory * {
    return new LincolnVehicleFactory();
  });
  Register(apollo::common::GEM, []() -> AbstractVehicleFactory * {
    return new GemVehicleFactory();
  });
  Register(apollo::common::LEXUS, []() -> AbstractVehicleFactory * {
    return new LexusVehicleFactory();
  });
  Register(apollo::common::TRANSIT, []() -> AbstractVehicleFactory * {
    return new TransitVehicleFactory();
  });
  Register(apollo::common::GE3, []() -> AbstractVehicleFactory * {
    return new Ge3VehicleFactory();
  });
  Register(apollo::common::WEY, []() -> AbstractVehicleFactory * {
    return new WeyVehicleFactory();
  });
  Register(apollo::common::ZHONGYUN, []() -> AbstractVehicleFactory * {
    return new ZhongyunVehicleFactory();
  });
  Register(apollo::common::CH, []() -> AbstractVehicleFactory * { 
    return new ChVehicleFactory(); 
  });
  Register(apollo::common::GUOXUAN，[]() ->AbstractVehicleFactory *{
    return new GuoxuanVehicleFactory();  //在此注册国轩车辆工厂
  })
}

//根据车辆参数创造一个车辆
std::unique_ptr<AbstractVehicleFactory> VehicleFactory::CreateVehicle(
    const VehicleParameter &vehicle_parameter) {
  auto abstract_factory = CreateObject(vehicle_parameter.brand()); //根据车辆品牌创造对象，返回指针
  //如果是空指针，!abstract_factory为真，则输出不能创建并输出调试字符串
  if (!abstract_factory) {
    AERROR << "failed to create vehicle factory with "
           << vehicle_parameter.DebugString();
  } else {//不是空指针，可以创建对象
    abstract_factory->SetVehicleParameter(vehicle_parameter); //设置车辆参数
    AINFO << "successfully created vehicle factory with " 
          << vehicle_parameter.DebugString();
  }
  return abstract_factory; //返回抽象工厂指针
}

}  // namespace canbus
}  // namespace apollo
