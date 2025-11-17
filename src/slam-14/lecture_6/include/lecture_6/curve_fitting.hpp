#pragma once
#include "interface/common/test_interface.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>

namespace VisualSLAM {
namespace Lecture6 {
    class CurveFittingTest: public FunctionTest::FunctionTestInterface {
    public:
        CurveFittingTest() : FunctionTest::FunctionTestInterface("curve_fitting_test_node") {
            // 添加测试函数
            add_test_function(std::bind(&CurveFittingTest::steepest_descent_curve_fitting, this));

            // 初始化绘图图像
            plot_image_ = cv::Mat::zeros(720, 1280, CV_8UC3);
            // 设置白色背景, 绘制坐标轴
            plot_image_.setTo(cv::Scalar(255, 255, 255));
            cv::line(plot_image_, cv::Point(0, plot_image_.rows * 0.8), cv::Point(plot_image_.cols, plot_image_.rows * 0.8), cv::Scalar(0, 0, 0), 1);
            cv::line(plot_image_, cv::Point(plot_image_.cols / 2, 0), cv::Point(plot_image_.cols / 2, plot_image_.rows), cv::Scalar(0, 0, 0), 1);
        }

    private:
        /**
         * @brief 最速下降法曲线拟合功能测试
         */
        int32_t steepest_descent_curve_fitting() {
            RCLCPP_INFO(logger_, "%s::%s 最速下降法曲线拟合测试开始",demangle(typeid(*this).name()).c_str(),  __func__);
            // 二次函数拟合 y = ax^2 + bx + c, 真实参数 a=-0.2, b=1.0, c=-1.25
            double real_a = -0.2, real_b = 1.0, real_c = -1.25;
            // 绘制真实曲线
            this->plot_curve_fitting_result(plot_image_, real_a, real_b, real_c, cv::Scalar(0, 0, 0), 1);
            // 生成带噪声的观测数据
            std::vector<double> x_data, y_data;
            cv::RNG rng;
            for (double x = -6.3; x <= 6.3; x += 0.1) {
                double noise = rng.gaussian(0.4);       // 生成随机高斯分布的噪声
                double y = real_a * x * x + real_b * x + real_c + noise;
                x_data.push_back(x);
                y_data.push_back(y);
            }
            // 绘制观测数据点
            this->plot_points(plot_image_, x_data, y_data, cv::Scalar(0, 255, 0));
            cv::imshow("Curve Fitting", plot_image_);
            cv::waitKey(0);

            // 最速下降法曲线拟合
            double a = 0.0, b = 0.0, c = 0.0;  // 初始化参数

            // 销毁窗口
            cv::destroyAllWindows();
            RCLCPP_INFO(logger_, "%s::%s 最速下降法曲线拟合测试结束",demangle(typeid(*this).name()).c_str(),  __func__);
            return EXIT_SUCCESS;
        }

        /**
         * @brief 牛顿法曲线拟合功能测试
         */
        int32_t newton_curve_fitting() {
            RCLCPP_INFO(logger_, "%s::%s 牛顿法曲线拟合测试开始",demangle(typeid(*this).name()).c_str(),  __func__);
            RCLCPP_INFO(logger_, "%s::%s 牛顿法曲线拟合测试结束",demangle(typeid(*this).name()).c_str(),  __func__);
            return EXIT_SUCCESS;
        }

        /**
         * @brief 高斯牛顿法曲线拟合功能测试
         */
        int32_t gauss_newton_curve_fitting() {
            RCLCPP_INFO(logger_, "%s::%s 高斯牛顿法曲线拟合测试开始",demangle(typeid(*this).name()).c_str(),  __func__);
            RCLCPP_INFO(logger_, "%s::%s 高斯牛顿法曲线拟合测试结束",demangle(typeid(*this).name()).c_str(),  __func__);
            return EXIT_SUCCESS;
        }

        /**
         * @brief 列文伯格-马夸尔特法曲线拟合功能测试
         */
        int32_t levenberg_marquardt_curve_fitting() {
            RCLCPP_INFO(logger_, "%s::%s 列文伯格-马夸尔特法曲线拟合测试开始",demangle(typeid(*this).name()).c_str(),  __func__);
            RCLCPP_INFO(logger_, "%s::%s 列文伯格-马夸尔特法曲线拟合测试结束",demangle(typeid(*this).name()).c_str(),  __func__);
            return EXIT_SUCCESS;
        }

        /**
         * @brief Ceres测试曲线拟合功能
         */
        int32_t ceres_curve_fitting() {
            RCLCPP_INFO(logger_, "%s::%s Ceres曲线拟合测试开始",demangle(typeid(*this).name()).c_str(),  __func__);
            RCLCPP_INFO(logger_, "%s::%s Ceres曲线拟合测试结束",demangle(typeid(*this).name()).c_str(),  __func__);
            return EXIT_SUCCESS;
        }

        /**
         * @brief G2O测试曲线拟合功能
         */
        int32_t g2o_curve_fitting() {
            RCLCPP_INFO(logger_, "%s::%s G2O曲线拟合测试开始",demangle(typeid(*this).name()).c_str(),  __func__);
            RCLCPP_INFO(logger_, "%s::%s G2O曲线拟合测试结束",demangle(typeid(*this).name()).c_str(),  __func__);
            return EXIT_SUCCESS;
        }

        /**
         * @brief 绘制曲线拟合结果
         */
        int32_t plot_curve_fitting_result(cv::Mat &plot_image, double a, double b, double c, cv::Scalar curve_color, int32_t point_size = 3) {
            RCLCPP_INFO(logger_, "%s::%s 绘制曲线拟合结果测试开始",demangle(typeid(*this).name()).c_str(),  __func__);
            std::vector<double> x_vals, y_vals;
            for (int col = 0; col < plot_image.cols; col++) {
                double x = (col - plot_image.cols / 2) * 0.01;  // 横坐标缩放因子
                double y = a * x * x + b * x + c;
                x_vals.push_back(x);
                y_vals.push_back(y);
            }
            // 绘制曲线
            this->plot_points(plot_image, x_vals, y_vals, curve_color, point_size);

            RCLCPP_INFO(logger_, "%s::%s 绘制曲线拟合结果测试结束",demangle(typeid(*this).name()).c_str(),  __func__);
            return EXIT_SUCCESS;
        }

        int32_t plot_points(cv::Mat &plot_image, const std::vector<double> &x_data, const std::vector<double> &y_data, cv::Scalar point_color, int32_t point_size = 3) {
            RCLCPP_INFO(logger_, "%s::%s 绘制数据点测试开始",demangle(typeid(*this).name()).c_str(),  __func__);
            for (size_t i = 0; i < x_data.size(); i++) {
                int col = static_cast<int>(x_data[i] * 100 + plot_image.cols / 2);  // 横坐标缩放因子
                int row = static_cast<int>(y_data[i] * 100 + plot_image.rows * 0.8);  // 纵坐标缩放因子
                if (row >= 0 && row < plot_image.rows && col >= 0 && col < plot_image.cols) {
                    cv::circle(plot_image, cv::Point(col, row), point_size, point_color, -1);
                }
            }
            RCLCPP_INFO(logger_, "%s::%s 绘制数据点测试结束",demangle(typeid(*this).name()).c_str(),  __func__);
            return EXIT_SUCCESS;
        }

    private:
        cv::Mat plot_image_;
    };
}   // namespace Lecture6
}   // namespace VisualSLAM