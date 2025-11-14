#include "interface/common/test_interface.hpp"
#include "lecture_4/sophus_test.hpp"

int32_t main(int32_t argc, char **argv) {
    rclcpp::init(argc, argv);

    // 运行 Sophus 测试
    FunctionTest::FunctionTestManager::iter()->test<VisualSLAM::Lecture4::SophusTest>();

    return 0;
}