#pragma once

#include <map>
#include <typeinfo>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "interface/common/singleton.hpp"

namespace FunctionTest {
    class FunctionTestInterface : public rclcpp::Node {
    public:
        FunctionTestInterface(const std::string &node_name) : rclcpp::Node(node_name), logger_(this->get_logger()) {
            // 获取当前包的路径
            if (package_name() != "") this->self_package_path_ = ament_index_cpp::get_package_share_directory(package_name());
        }

        virtual ~FunctionTestInterface() {
            RCLCPP_INFO(logger_, "销毁测试类实例, 类%s", demangle(typeid(*this).name()).c_str());
        }

        int32_t test() {
            RCLCPP_INFO(this->get_logger(), "启动测试, 类%s", demangle(typeid(*this).name()).c_str());
            for (const auto &item : test_function_list_) {
                int32_t ret = item();
                if (ret != EXIT_SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "测试类%s中的测试函数失败, 返回值: %d", demangle(typeid(*this).name()).c_str(), ret);
                    return ret;
                }
            }
            RCLCPP_INFO(this->get_logger(), "测试类%s所有测试函数通过.", demangle(typeid(*this).name()).c_str());
            return EXIT_SUCCESS;
        }

    protected:
        std::string package_name () {
            std::string name = "";
#ifdef PACKAGE_NAME
            name = PACKAGE_NAME;
#endif
            return name;
        }

        /**
         * @brief 添加测试函数
         * @param func 测试函数
         */
        void add_test_function(const std::function<int32_t()> &func) {
            test_function_list_.emplace_back(func);
        }

        std::string demangle(const char* name) {
            int status = -1;
            std::unique_ptr<char, void(*)(void*)> res{
                abi::__cxa_demangle(name, nullptr, nullptr, &status),
                std::free
            };
            return (status == 0) ? res.get() : name;
        }

    protected:
        rclcpp::Logger logger_;

        std::string self_package_path_;
        std::list<std::function<int32_t()>> test_function_list_;
    };

    /**
     * @brief 测试管理, 这里将模块配置好, 外部直接通过单例测试制定模块
     */
    class FunctionTestManager : public Interface::common::Singleton<FunctionTestManager> {
        friend class Interface::common::Singleton<FunctionTestManager>;

    public:
        ~FunctionTestManager() = default;

        template<typename T>
        std::shared_ptr<T> test() {
            std::shared_ptr<T> test_class = std::make_shared<T>();
            test_class->test();
            return test_class;
        }

    protected:
        FunctionTestManager() = default;

    };
}