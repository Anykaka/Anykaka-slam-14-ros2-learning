#pragma once
#include "interface/common/test_interface.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace VisualSLAM {
namespace Lecture7 {
    class FeatureExtractionMatchingTest: public FunctionTest::FunctionTestInterface {
    public:
        FeatureExtractionMatchingTest() : FunctionTest::FunctionTestInterface("feature_extraction_matching_test_node") {
            // 添加测试函数
            // add_test_function(std::bind(&FeatureExtractionMatchingTest::test_function_name, this));
        }
    };
}   // namespace Lecture7
}   // namespace VisualSLAM