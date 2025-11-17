#include "interface/common/test_interface.hpp"
#include "lecture_5/camera_image.hpp"
#include "lecture_5/3d_vision.hpp"

int32_t main(int32_t argc, char **argv) {
    // 初始化RCLCPP并实例化多线程执行器
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    // 运行 CameraImage 测试
    // auto camera_image_test = FunctionTest::FunctionTestManager::iter()->test<VisualSLAM::Lecture5::CameraImageTest>();
    // executor.add_node(camera_image_test);

    // 运行 Vision3D 测试
    auto vision_3d_test = FunctionTest::FunctionTestManager::iter()->test<VisualSLAM::Lecture5::Vision3DTest>();
    executor.add_node(vision_3d_test);

    // executor.spin();
    rclcpp::shutdown();
    return 0;
}