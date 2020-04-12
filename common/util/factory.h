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
 * @brief Defines the Factory class.
 * 定义Factory类
 */

#pragma once

#include <map>
#include <memory>
#include <utility>

#include "cyber/common/macros.h"

#include "cyber/common/log.h"

/**
 * @namespace apollo::common::util
 * @brief apollo::common::util
 */
namespace apollo {
namespace common {
namespace util {

/**
 * @class Factory类
 * @brief Implements a Factory design pattern with Register and Create methods
 *
 * The objects created by this factory all implement the same interface
 * (namely, AbstractProduct). This design pattern is useful in settings where
 * multiple implementations of an interface are available, and one wishes to
 * defer the choice of the implementation in use.
 * 使用Register和Create方法实现Factory设计模式
   该工厂创建的对象都实现相同的接口（即AbstractProduct）。
   在一个接口的多个实现可用的设置中，该设计模式非常有用，并且希望延迟选择所使用的实现。
 *
 * @param IdentifierType Type used for identifying the registered classes,
 * typically std::string.  
 * 用于识别注册类别的类型，通常是std :: string。
 * @param AbstractProduct The interface implemented by the registered classes
 * 由注册类实现的接口
 * @param ProductCreator Function returning a pointer to an instance of
 * the registered class
 * 函数返回指向注册类实例的指针
 * @param MapContainer Internal implementation of the function mapping
 * IdentifierType to ProductCreator, by default std::unordered_map
 * 将IdentifierType映射到ProductCreator的函数的内部实现，默认情况下为std :: unordered_map
 */
template <typename IdentifierType, class AbstractProduct,
          class ProductCreator = AbstractProduct *(*)(),
          class MapContainer = std::map<IdentifierType, ProductCreator>>
class Factory {
 public:
  /**
   * @brief Registers the class given by the creator function, linking it to id.
   * Registration must happen prior to calling CreateObject.
   * 注册creator函数提供的类，并将其链接到id。 注册必须在调用CreateObject之前进行。
   * @param id Identifier of the class being registered
   * 正在注册的类别的标识符
   * @param creator Function returning a pointer to an instance of
   * the registered class
   * 函数返回指向注册类实例的指针
   * @return True iff the key id is still available
   * 如果密钥ID仍然可用，则为true
   */
  bool Register(const IdentifierType &id, ProductCreator creator) {
    return producers_.insert(std::make_pair(id, creator)).second; //插入
  }

  //能否找到id
  bool Contains(const IdentifierType &id) {
    return producers_.find(id) != producers_.end();
  }

  /**
   * @brief Unregisters the class with the given identifier
   * 使用给定的标识符注销该类
   * @param id The identifier of the class to be unregistered
   * 要取消注册的类的标识符
   */
  //如果注销掉了则返回true
  bool Unregister(const IdentifierType &id) {
    return producers_.erase(id) == 1; //注销掉了producers_.erase(id)=1
  }

  //清理缓存之类
  void Clear() { producers_.clear(); }

  //再次检查是否已经在producers_里注销掉，注销掉了为空返回true
  bool Empty() const { return producers_.empty(); }

  /**
   * @brief Creates and transfers membership of an object of type matching id.
   * Need to register id before CreateObject is called. May return nullptr
   * silently.
   * 创建并传输ID匹配类型的对象的成员身份。需要在调用CreateObject之前注册ID。可能会静默返回nullptr。
   * @param id The identifier of the class we which to instantiate
   * 我们要实例化的类的标识符
   * @param args the object construction arguments
   * 对象构造的参数
   */
  template <typename... Args> //可变参数模板，参数类型和数量都可以改变
  std::unique_ptr<AbstractProduct> CreateObjectOrNull(const IdentifierType &id,
                                                      Args &&... args) {
    auto id_iter = producers_.find(id); //找到id的迭代器
    if (id_iter != producers_.end()) {
      return std::unique_ptr<AbstractProduct>(
          (id_iter->second)(std::forward<Args>(args)...)); //返回创建的对象的指针
    }
    return nullptr; //返回空指针
  }

  /**
   * @brief Creates and transfers membership of an object of type matching id.
   * Need to register id before CreateObject is called.
   * 创建并传输ID匹配类型的对象的成员身份。需要在调用CreateObject之前注册ID。
   * @param id The identifier of the class we which to instantiate
   * 我们要实例化的类的标识符
   * @param args the object construction arguments
   * 对象构造的参数
   */
  template <typename... Args>
  std::unique_ptr<AbstractProduct> CreateObject(const IdentifierType &id,
                                                Args &&... args) {
    auto result = CreateObjectOrNull(id, std::forward<Args>(args)...); //返回指针
    //result为空指针时，!result为真，输出不能创建这种类型的对象：id
    AERROR_IF(!result) << "Factory could not create Object of type : " << id;
    return result; //返回result指针
  }

 private:
  MapContainer producers_; //将IdentifierType映射到ProductCreator的函数的内部实现
};

}  // namespace util
}  // namespace common
}  // namespace apollo
