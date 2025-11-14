#pragma once

#include "interface/common/test_interface.hpp"

namespace VisualSLAM {
namespace Lecture4 {
    class SophusTest: public FunctionTest::FunctionTestInterface {
    public:
        SophusTest() : FunctionTest::FunctionTestInterface("sophus_test_node") {}

        int32_t test() override {
            RCLCPP_INFO(this->get_logger(), "Running Sophus Test...");

            // Sophus 测试代码放在这里
            // ...

            RCLCPP_INFO(this->get_logger(), "Sophus Test Completed.");
            return 0;
        }
    private:

    };

}   // namespace Lecture4
}   // namespace VisualSLAM