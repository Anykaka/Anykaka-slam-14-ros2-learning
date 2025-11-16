#pragma once
#include "interface/common/test_interface.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>

namespace VisualSLAM {
namespace Lecture5 {
    class CameraImageTest: public FunctionTest::FunctionTestInterface {
    public:
        CameraImageTest() : FunctionTest::FunctionTestInterface("camera_image_test_node") {
            // 添加测试函数
            // add_test_function(std::bind(&CameraImageTest::opencv_basic_test, this));
            add_test_function(std::bind(&CameraImageTest::undistort_image_test, this));
            add_test_function(std::bind(&CameraImageTest::undistort_image_by_opencv_test, this));
        }

    private:
        /**
         * @brief OpenCV基础功能测试
         */
        int32_t opencv_basic_test() {
            RCLCPP_INFO(logger_, "%s::%s OpenCV基础用法开始测试",demangle(typeid(*this).name()).c_str(),  __func__);
            // 加载图像
            cv::Mat ubuntu_image = cv::imread(self_package_path_ + "/config/ubuntu.png");
            if (ubuntu_image.empty()) {
                RCLCPP_INFO(logger_, "图像 %s/config/ubuntu.png 加载失败", self_package_path_.c_str());
            }

            RCLCPP_INFO(logger_, "图像 %s/config/ubuntu.png 信息: %d x %d, 类型: %s, 通道数: %d",
                        self_package_path_.c_str(),
                        ubuntu_image.rows, ubuntu_image.cols,
                        ubuntu_image.type() == CV_8UC3 ? "CV_8UC3" : "其他类型",
                        ubuntu_image.channels());

            cv::imshow("Ubuntu Logo", ubuntu_image);
            cv::waitKey(1000);

            // 图像遍历耗时测试
            char total_pixels[ubuntu_image.rows * ubuntu_image.cols * ubuntu_image.channels()] = {0};
            auto start = std::chrono::high_resolution_clock::now();
            for (int32_t row_index =  0; row_index < ubuntu_image.rows; ++row_index) {
                for (int32_t col_index = 0; col_index < ubuntu_image.cols; ++col_index) {
                    for (int32_t channel_index = 0; channel_index < ubuntu_image.channels(); ++channel_index) {
                        total_pixels[row_index * ubuntu_image.cols * ubuntu_image.channels() + col_index * ubuntu_image.channels() + channel_index] =
                            ubuntu_image.at<cv::Vec3b>(row_index, col_index)[channel_index];
                    }
                }
            }
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            RCLCPP_INFO(logger_, "图像遍历耗时: %ld 毫秒", duration.count());

            // 像素修改及浅拷贝
            cv::Mat copy_ubuntu_image = ubuntu_image;
            copy_ubuntu_image(cv::Rect(0, 0, copy_ubuntu_image.cols, copy_ubuntu_image.rows / 2)) = cv::Scalar(0, 0, 255);
            cv::imshow("Ubuntu Logo", ubuntu_image);
            cv::imshow("Modified Copy Ubuntu Logo", copy_ubuntu_image);
            cv::waitKey(1000);

            // 像素修改及深拷贝
            cv::Mat deep_copy_ubuntu_image = ubuntu_image.clone();
            deep_copy_ubuntu_image(cv::Rect(0, 0, deep_copy_ubuntu_image.cols, deep_copy_ubuntu_image.rows / 2)) = cv::Scalar(255, 0, 0);
            cv::imshow("Ubuntu Logo", ubuntu_image);
            cv::imshow("Modified Copy Ubuntu Logo", deep_copy_ubuntu_image);
            cv::waitKey(1000);

            // 清理窗口
            cv::destroyAllWindows();
            RCLCPP_INFO(logger_, "%s::%s OpenCV基础用法结束测试",demangle(typeid(*this).name()).c_str(),  __func__);
            return EXIT_SUCCESS;
        }

        /**
         * @brief 图像去畸变
         */
        int32_t undistort_image_test() {
            RCLCPP_INFO(logger_, "%s::%s OpenCV图像去畸变开始测试",demangle(typeid(*this).name()).c_str(),  __func__);

            // 读取畸变图像
            cv::Mat distort_image = cv::imread(self_package_path_ + "/config/distorted.png", cv::ImreadModes::IMREAD_GRAYSCALE);
            if (distort_image.empty()) {
                RCLCPP_INFO(logger_, "图像 %s/config/distorted.png 加载失败", self_package_path_.c_str());
            }
            cv::imshow("Distorted Image", distort_image);
            cv::waitKey(1000);

            // 相机内参
            double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;
            // 畸变参数
            double k1 = -0.28340811, k2 = 0.07395908, k3 = 0.0, p1 = 0.00019359, p2 = 1.76187114e-5;
            // 畸变矫正图像
            cv::Mat undistort_image = cv::Mat::zeros(distort_image.size(), distort_image.type());

            // 畸变矫正
            for (int32_t v = 0; v < distort_image.rows; ++v) {
                for (int32_t u = 0; u < distort_image.cols; ++u) {
                    // 归一化平面坐标
                    double x = (u - cx) / fx;
                    double y = (v - cy) / fy;

                    double r2 = x * x + y * y;
                    double r4 = r2 * r2;
                    double r6 = r4 * r2;

                    // 径向畸变
                    double x_distort = x * (1 + k1 * r2 + k2 * r4 + k3 * r6);
                    double y_distort = y * (1 + k1 * r2 + k2 * r4 + k3 * r6);

                    // 切向畸变
                    x_distort += 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
                    y_distort += p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;

                    // 像素坐标
                    int32_t u_distort = static_cast<int32_t>(fx * x_distort + cx);
                    int32_t v_distort = static_cast<int32_t>(fy * y_distort + cy);

                    // 像素赋值
                    if (u_distort >= 0 && u_distort < distort_image.cols &&
                        v_distort >= 0 && v_distort < distort_image.rows) {
                        undistort_image.at<uchar>(v, u) = distort_image.at<uchar>(v_distort, u_distort);
                    }
                }
            }
            cv::imshow("Undistorted Image", undistort_image);
            cv::imshow("Distorted Image", distort_image);
            cv::waitKey(1000);

            // 清理窗口
            cv::destroyAllWindows();
            RCLCPP_INFO(logger_, "%s::%s OpenCV图像去畸变结束测试",demangle(typeid(*this).name()).c_str(),  __func__);
            return EXIT_SUCCESS;
        }

        /**
         * @brief OpenCV图像去畸变
         */
        int32_t undistort_image_by_opencv_test() {
            RCLCPP_INFO(logger_, "%s::%s OpenCV图像去畸变开始测试",demangle(typeid(*this).name()).c_str(),  __func__);
            // 读取畸变图像
            cv::Mat distort_image = cv::imread(self_package_path_ + "/config/distorted.png", cv::ImreadModes::IMREAD_GRAYSCALE);
            if (distort_image.empty()) {
                RCLCPP_INFO(logger_, "图像 %s/config/distorted.png 加载失败", self_package_path_.c_str());
            }
            cv::imshow("Distorted Image", distort_image);
            cv::waitKey(1000);

            // 相机内参
            double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;
            // 畸变参数
            double k1 = -0.28340811, k2 = 0.07395908, k3 = 0.0, p1 = 0.00019359, p2 = 1.76187114e-5;

            // OpenCV畸变矫正
            cv::Mat undistort_image = cv::Mat::zeros(distort_image.size(), distort_image.type());
            cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
                                      fx, 0, cx,
                                      0, fy, cy,
                                      0, 0, 1);
            cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
            cv::undistort(distort_image, undistort_image, camera_matrix, dist_coeffs);

            cv::imshow("Undistorted Image", undistort_image);
            cv::imshow("Distorted Image", distort_image);
            cv::waitKey(0);

            // 清理窗口
            cv::destroyAllWindows();
            RCLCPP_INFO(logger_, "%s::%s OpenCV图像去畸变结束测试",demangle(typeid(*this).name()).c_str(),  __func__);
            return EXIT_SUCCESS;
        }

    };
}   // namespace Lecture5
}   // namespace VisualSLAM