#include "interface/common/test_interface.hpp"

int32_t main(int32_t argc, char **argv) {
    // 初始化RCLCPP并实例化多线程执行器
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

#if 0
    // 运行 CurveFitting 测试
    // auto curve_fitting_test = FunctionTest::FunctionTestManager::iter()->test<VisualSLAM::Lecture6::CurveFittingTest>();
    // executor.add_node(curve_fitting_test);

#else
    (void) executor;
#endif

    // executor.spin();
    rclcpp::shutdown();

    return 0;
}