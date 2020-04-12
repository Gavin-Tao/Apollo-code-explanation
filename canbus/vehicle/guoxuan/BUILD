//guoxuan编译文件


load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "guoxuan_vehicle_factory",
    srcs = ["guoxuan_vehicle_factory.cc"],  //添加源文件guoxuan_vehicle_factory.cc
    hdrs = ["guoxuan_vehicle_factory.h"],   //添加头文件guoxuan_vehicle_factory.h
    deps = [
        ":guoxuan_controller",  //添加依赖项guoxuan_controller
        ":guoxuan_message_manager",  //添加依赖项guoxuan_message_manager
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "guoxuan_message_manager",  //添加消息管理姓名guoxuan_message_manager
    srcs = ["guoxuan_message_manager.cc"], //添加源文件guoxuan_message_manager.cc
    hdrs = ["guoxuan_message_manager.h"],  //添加头文件guoxuan_message_manager.h
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle/guoxuan/protocol:canbus_guoxuan_protocol",  //添加依赖项protocol:canbus_guoxuan_protocol
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_library(
    name = "guoxuan_controller",  //添加国轩控制器姓名guoxuan_controller
    srcs = ["guoxuan_controller.cc"],  //添加源文件guoxuan_controller.cc
    hdrs = ["guoxuan_controller.h"],  //添加头文件guoxuan_controller.h
    deps = [
        ":guoxuan_message_manager",  //添加依赖项guoxuan_message_manager
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/guoxuan/protocol:canbus_zhongyun_protocol", //添加依赖项guoxuan/protocol:canbus_zhongyun_protocol
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

//暂时还未编写test文件
cc_test(
    name = "guoxuan_message_manager_test", //添加国轩消息管理器测试姓名guoxuan_message_manager_test
    size = "small",
    srcs = ["guoxuan_message_manager_test.cc"], //添加源文件guoxuan_message_manager_test.cc
    data = ["//modules/canbus:canbus_testdata"], 
    deps = [
        ":guoxuan_message_manager",  //添加依赖项guoxuan_message_manager
        "@gtest//:main",
    ],
)

cc_test(
    name = "guoxuan_vehicle_factory_test", //添加国轩车辆工厂测试姓名guoxuan_vehicle_factory_test
    size = "small",
    srcs = ["guoxuan_vehicle_factory_test.cc"],  //添加源文件guoxuan_vehicle_factory_test.cc
    data = ["//modules/canbus:canbus_testdata"],
    deps = [
        ":guoxuan_vehicle_factory", //添加依赖项guoxuan_vehicle_factory
        "@gtest//:main",
    ],
)

cc_test(
    name = "guoxuan_controller_test",  //添加国轩控制器测试姓名guoxuan_controller_test
    size = "small",
    srcs = ["guoxuan_controller_test.cc"],  //添加源文件guoxuan_controller_test.cc
    data = ["//modules/canbus:canbus_testdata"],
    deps = [
        ":guoxuan_controller",  //添加依赖项guoxuan_controller
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cpplint()
