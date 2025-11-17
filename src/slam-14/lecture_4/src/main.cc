#include "interface/common/test_interface.hpp"
#include "lecture_4/sophus_test.hpp"

int32_t main(int32_t argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

#if 0
    // 运行 Sophus 测试
    // auto sophus_test = FunctionTest::FunctionTestManager::iter()->test<VisualSLAM::Lecture4::SophusTest>();
    // executor.add_node(sophus_test);
#else
    (void) executor;
#endif

    // executor.spin();
    rclcpp::shutdown();

    return 0;
}