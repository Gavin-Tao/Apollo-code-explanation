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

#include "modules/canbus/common/canbus_gflags.h"

// System gflags  系统gflags
//语法：DEFINE_type(变量名, 默认值, "介绍")
//在程序中就可以通过FLAGS_name的形式访问对应的flags参数了。
//FLAGS_是固定的前缀，name为宏定义中使用的名字。
DEFINE_string(canbus_node_name, "chassis", "The chassis module name in proto");//底盘节点名称
DEFINE_string(canbus_module_name, "canbus_component", "Module name");//canbus组件

// data file
DEFINE_string(canbus_conf_file,
              "/apollo/modules/canbus/conf/canbus_conf.pb.txt",
              "Default canbus conf file");//默认的canbus conf文件

// Canbus gflags
DEFINE_double(chassis_freq, 100, "Chassis feedback timer frequency."); //底盘反馈定时器的频率100Hz
DEFINE_int64(min_cmd_interval, 5, "Minimum control command interval in ms."); //最小控制命令间隔ms

// chassis_detail message publish 底盘信息发布
DEFINE_bool(enable_chassis_detail_pub, false, "Chassis Detail message publish");//底盘信息默认不发布

// canbus test files
DEFINE_string(canbus_test_file,
              "/apollo/modules/canbus/testdata/canbus_test.pb.txt",
              "canbus tester input test file, in ControlCommand pb format.");
//canbus测试器输入测试文件，格式为ControlCommand pb

// enable receiving guardian command 确保能够收到导航指令
// TODO(QiL) : depreciate this after test  测试后折旧/贬低
DEFINE_bool(receive_guardian, false,
            "Enable receiving guardian message on canbus side");//在canbus端确保能收到导航指令

DEFINE_int32(guardian_cmd_pending_queue_size, 10,
             "Max guardian cmd pending queue size"); // cmd=command，导航命令待处理的队列大小最大值
DEFINE_int32(control_cmd_pending_queue_size, 10,
             "Max control cmd pending queue size"); //控制命令待处理的队列大小最大值
