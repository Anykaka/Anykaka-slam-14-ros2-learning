#pragma once
#include "interface/common/test_interface.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace VisualSLAM {
namespace Lecture5 {
    class CurveFittingTest: public FunctionTest::FunctionTestInterface {
    public:
        CurveFittingTest() : FunctionTest::FunctionTestInterface("curve_fitting_test_node") {
            // 添加测试函数
            add_test_function(std::bind(&CurveFittingTest::test_curve_fitting, this));
        }

    private:
        /**
         * @brief 测试曲线拟合功能
         */
        int32_t test_curve_fitting() {
            return EXIT_SUCCESS;
        }
    };
}   // namespace Lecture5
}   // namespace VisualSLAM