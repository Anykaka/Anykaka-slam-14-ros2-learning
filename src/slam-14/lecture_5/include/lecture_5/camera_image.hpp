#pragma once
#include "interface/common/test_interface.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace VisualSLAM {
namespace Lecture5 {
    class CameraImageTest: public FunctionTest::FunctionTestInterface {
    public:
        CameraImageTest() : FunctionTest::FunctionTestInterface("camera_image_test_node") {
            // 添加测试函数
            add_test_function(std::bind(&CameraImageTest::test_camera_image_loading, this));
        }

    private:
        /**
         * @brief 测试相机图像加载功能
         */
        int32_t test_camera_image_loading() {
            return EXIT_SUCCESS;
        }
    };
}   // namespace Lecture5
}   // namespace VisualSLAM