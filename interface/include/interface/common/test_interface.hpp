#pragma once

#include <map>
#include <typeinfo>
#include <rclcpp/rclcpp.hpp>

#include "interface/common/singleton.hpp"

namespace FunctionTest {
    class FunctionTestInterface : public rclcpp::Node {
    public:
        FunctionTestInterface(const std::string &node_name) : rclcpp::Node(node_name) {}

        virtual int32_t test() = 0;

    protected:

    };

    /**
     * @brief 测试管理, 这里将模块配置好, 外部直接通过单例测试制定模块
     */
    class FunctionTestManager : public Interface::common::Singleton<FunctionTestManager> {
        friend class Interface::common::Singleton<FunctionTestManager>;

    public:
        ~FunctionTestManager() = default;

        template<typename T>
        int32_t test() {
            std::cout << "Testing module: " << typeid(T).name() << std::endl;

            T test_class;
            return test_class.test();
        }

    protected:
        FunctionTestManager() = default;

    };
}