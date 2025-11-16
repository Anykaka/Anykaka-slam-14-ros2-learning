#include "interface/common/test_interface.hpp"

int32_t main(int32_t argc, char **argv) {
    rclcpp::init(argc, argv);

    // 运行 Sophus 测试
    // FunctionTest::FunctionTestManager::iter()->test<VisualSLAM::Lecture4::SophusTest>();

    rclcpp::shutdown();

    return 0;
}