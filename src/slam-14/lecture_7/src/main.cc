#include "interface/common/test_interface.hpp"
#include "lecture_7/feature_extracte_matching.hpp"

int32_t main(int32_t argc, char **argv) {
    // 初始化RCLCPP并实例化多线程执行器
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

#if 1
    // 运行 FeatureExtractionMatching 测试
    auto feature_extraction_matching_test = FunctionTest::FunctionTestManager::iter()->test<VisualSLAM::Lecture7::FeatureExtractionMatchingTest>();
    executor.add_node(feature_extraction_matching_test);

#else
    (void) executor;
#endif

    // executor.spin();
    rclcpp::shutdown();

    return 0;
}