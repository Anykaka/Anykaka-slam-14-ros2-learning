#pragma once
#include "interface/common/test_interface.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>

namespace VisualSLAM {
namespace Lecture5 {
    class Vision3DTest: public FunctionTest::FunctionTestInterface {
    public:
        Vision3DTest() : FunctionTest::FunctionTestInterface("vision_3d_test_node") {
            // 添加测试函数
        }

    private:
        /**
         * @brief 双目相机
         */
        int32_t stereo_vision_test() {
            RCLCPP_INFO(logger_, "%s::%s 双目相机测试开始",demangle(typeid(*this).name()).c_str(),  __func__);
            // TODO 双目相机测试代码实现

            RCLCPP_INFO(logger_, "%s::%s 双目相机测试结束",demangle(typeid(*this).name()).c_str(),  __func__);
            return EXIT_SUCCESS;
        }

        /**
         * @brief RGB-D相机
         */
        int32_t rgbd_vision_test() {
            RCLCPP_INFO(logger_, "%s::%s RGB-D相机测试开始",demangle(typeid(*this).name()).c_str(),  __func__);
            // TODO RGB-D相机测试代码实现

            RCLCPP_INFO(logger_, "%s::%s RGB-D相机测试结束",demangle(typeid(*this).name()).c_str(),  __func__);
            return EXIT_SUCCESS;
        }
    };
}   // namespace Lecture5
}   // namespace VisualSLAM