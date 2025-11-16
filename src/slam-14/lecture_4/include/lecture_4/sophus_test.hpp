#pragma once

#include "interface/common/test_interface.hpp"
#include <Eigen/Eigen>
#include <sophus/se3.hpp>
#include <fstream>
#include <iostream>

namespace VisualSLAM {
namespace Lecture4 {
    class SophusTest: public FunctionTest::FunctionTestInterface {
    public:
        SophusTest() : FunctionTest::FunctionTestInterface("sophus_test_node") {
            // 添加测试函数
            add_test_function(std::bind(&SophusTest::base_use, this));
            add_test_function(std::bind(&SophusTest::compute_trajectory_error, this));
        }

    private:
        /**
         * @brief Sophus 基础用法测试
         */
        int32_t base_use() {
            RCLCPP_INFO(this->get_logger(), "%s::%s 基础用法测试", demangle(typeid(*this).name()).c_str(), __FUNCTION__);

            // 使用 Eigen 进行旋转矩阵的创建
            Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            Eigen::Quaterniond q(R);        // 通过旋转矩阵创建四元数
            Sophus::SO3d SO3_R(R);          // 通过旋转矩阵构造正交群
            Sophus::SO3d SO3_q(q);          // 通过四元数构造正交群

            RCLCPP_INFO_STREAM(this->get_logger(), "旋转矩阵:\n" << R);
            RCLCPP_INFO_STREAM(this->get_logger(), "四元数:\n" << q.coeffs().transpose());
            RCLCPP_INFO_STREAM(this->get_logger(), "R矩阵构造的SO3:\n" << SO3_R.matrix());
            RCLCPP_INFO_STREAM(this->get_logger(), "q四元数构造的SO3:\n" << SO3_q.matrix());
            
            Eigen::Vector3d so3 = SO3_R.log();   // 对数映射得到李代数
            RCLCPP_INFO_STREAM(this->get_logger(), "SO3的李代数:\n" << so3.transpose());
            // 指数映射还原旋转矩阵
            Eigen::Matrix3d fai_hat = Sophus::SO3d::hat(so3);
            // 正交代数的反对称矩阵Φ^
            RCLCPP_INFO_STREAM(this->get_logger(), "so3的hat:\n" << fai_hat);
            // 正交代数的反对称矩阵
            Eigen::Vector3d fai_hat_vee = Sophus::SO3d::vee(fai_hat);
            RCLCPP_INFO_STREAM(this->get_logger(), "fai_hat的vee:\n" << fai_hat_vee.transpose());

            // BCH 扰动模型, 左乘微小变量
            Eigen::Vector3d delta_so3(1e-4, 0, 0);   // 微小扰动
            Sophus::SO3d SO3_updated = Sophus::SO3d::exp(delta_so3) * SO3_R;    // 将扰动变量转换为正交群后左乘更新
            RCLCPP_INFO_STREAM(this->get_logger(), "扰动矩阵:\n" << Sophus::SO3d::exp(delta_so3).matrix());
            RCLCPP_INFO_STREAM(this->get_logger(), "更新后的SO3:\n" << SO3_updated.matrix());

            // 欧式群
            Eigen::Vector3d t(1, 0, 0);    // 平移向量
            Sophus::SE3d SE3_Rt(R, t);     // 通过旋转矩阵和平移向量构造欧式群
            Sophus::SE3d SE3_qt(q, t);     // 通过四元数和平移向量构造欧式群
            RCLCPP_INFO_STREAM(this->get_logger(), "R和t构造的SE3:\n" << SE3_Rt.matrix());
            RCLCPP_INFO_STREAM(this->get_logger(), "q和t构造的SE3:\n" << SE3_qt.matrix());

            // 欧式群的李代数
            Eigen::Matrix<double, 6, 1> se3 = SE3_Rt.log();
            RCLCPP_INFO_STREAM(this->get_logger(), "SE3的李代数:\n" << se3.transpose());
            // 欧式群李代数的反对称矩阵
            Eigen::Matrix4d se3_hat = Sophus::SE3d::hat(se3);
            RCLCPP_INFO_STREAM(this->get_logger(), "se3的hat:\n" << se3_hat);
            // 欧式群李代数的反对称矩阵转向量
            Eigen::Matrix<double, 6, 1> se3_vee = Sophus::SE3d::vee(se3_hat);
            RCLCPP_INFO_STREAM(this->get_logger(), "se3_hat的vee:\n" << se3_vee.transpose());

            // 欧式群的BCH扰动模型, 左乘微小变量
            Eigen::Matrix<double, 6, 1> delta_se3;
            delta_se3 << 1e-4, 0, 0, 0, 0, 0;   // 微小扰动
            Sophus::SE3d SE3_updated = Sophus::SE3d::exp(delta_se3) * SE3_Rt;    // 将扰动变量转换为欧式群后左乘更新
            RCLCPP_INFO_STREAM(this->get_logger(), "扰动矩阵:\n" << Sophus::SE3d::exp(delta_se3).matrix());
            RCLCPP_INFO_STREAM(this->get_logger(), "更新后的SE3:\n" << SE3_updated.matrix());

            RCLCPP_INFO(this->get_logger(), "%s::%s基础用法测试完成.", demangle(typeid(*this).name()).c_str(), __FUNCTION__);
            return EXIT_SUCCESS;
        }

        int32_t compute_trajectory_error() {
            RCLCPP_INFO(this->get_logger(), "%s::%s 轨迹误差计算测试", demangle(typeid(*this).name()).c_str(), __FUNCTION__);

            // 获取轨迹文件路径
            std::string estimated_trajectory_file = this->self_package_path_ + "/config/estimated.txt";
            std::string groundtruth_trajectory_file = this->self_package_path_ + "/config/groundtruth.txt";

            std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> estimated_trajectory;
            std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> groundtruth_trajectory;
            auto estimated_ret = read_trajectory_file(estimated_trajectory_file, estimated_trajectory);
            if (estimated_ret != EXIT_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "读取估计轨迹文件%s失败.", estimated_trajectory_file.c_str());
                return estimated_ret;
            }
            auto groundtruth_ret = read_trajectory_file(groundtruth_trajectory_file, groundtruth_trajectory);
            if (groundtruth_ret != EXIT_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "读取真实轨迹文件%s失败.", groundtruth_trajectory_file.c_str());
            }
            if (estimated_trajectory.size() != groundtruth_trajectory.size() || estimated_trajectory.empty() || groundtruth_trajectory.empty()) {
                RCLCPP_ERROR(this->get_logger(), "估计轨迹与真实轨迹长度不匹配或存在轨迹为空.");
                return EXIT_FAILURE;
            }

            RCLCPP_INFO(this->get_logger(), "读取轨迹文件成功, 轨迹点数量: %zu, 开始计算误差...", estimated_trajectory.size());
            // 计算绝对轨迹误差 Absolute Trajectory Error (ATE)
            double absolute_translation_error = 0.0;
            for (size_t i = 0; i < estimated_trajectory.size(); ++i) {
                Sophus::SE3d error_transform = groundtruth_trajectory[i].inverse() * estimated_trajectory[i];
                double error = error_transform.log().norm();
                absolute_translation_error += error * error;
                // RCLCPP_INFO_STREAM(this->get_logger(), "第 " << i << " 组位姿[(" << estimated_trajectory[i].translation().transpose() << "), (" << groundtruth_trajectory[i].translation().transpose() << ")]误差: " << error);
            }
            absolute_translation_error = std::sqrt(absolute_translation_error / estimated_trajectory.size());
            RCLCPP_INFO(this->get_logger(), "绝对轨迹误差 ATE: %f", absolute_translation_error);

            // 计算绝对平移误差 Average Translational Error (ATE)
            double averge_translation_error = 0.0;
            for (size_t i = 0; i < estimated_trajectory.size(); ++i) {
                Eigen::Vector3d error = groundtruth_trajectory[i].translation() - estimated_trajectory[i].translation();
                averge_translation_error += error.squaredNorm();
            }
            averge_translation_error = std::sqrt(averge_translation_error / estimated_trajectory.size());
            RCLCPP_INFO(this->get_logger(), "绝对平移误差 ATE: %f", averge_translation_error);

            // 计算相对位姿误差 Relative Pose Error (RPE)
            double relative_pose_error = 0.0;
            for (size_t i = 0; i < estimated_trajectory.size() - 1; ++i) {
                Sophus::SE3d estimated_relative = estimated_trajectory[i].inverse() * estimated_trajectory[i + 1];
                Sophus::SE3d groundtruth_relative = groundtruth_trajectory[i].inverse() * groundtruth_trajectory[i + 1];
                Sophus::SE3d error_transform = groundtruth_relative.inverse() * estimated_relative;
                double error = error_transform.log().norm();
                relative_pose_error += error * error;
            }
            relative_pose_error = std::sqrt(relative_pose_error / (estimated_trajectory.size() - 1));
            RCLCPP_INFO(this->get_logger(), "相对位姿误差 RPE: %f", relative_pose_error);

            // 计算相对平移误差 Average Relative Translational Error (RTE)
            double average_relative_translation_error = 0.0;
            for (size_t i = 0; i < estimated_trajectory.size() - 1; ++i) {
                Eigen::Vector3d estimated_relative = estimated_trajectory[i + 1].translation() - estimated_trajectory[i].translation();
                Eigen::Vector3d groundtruth_relative = groundtruth_trajectory[i + 1].translation() - groundtruth_trajectory[i].translation();
                Eigen::Vector3d error = groundtruth_relative - estimated_relative;
                average_relative_translation_error += error.squaredNorm();
            }
            average_relative_translation_error = std::sqrt(average_relative_translation_error / (estimated_trajectory.size() - 1));
            RCLCPP_INFO(this->get_logger(), "相对平移误差 RTE: %f", average_relative_translation_error);

            RCLCPP_INFO(this->get_logger(), "%s::%s 轨迹误差计算测试完成", demangle(typeid(*this).name()).c_str(), __FUNCTION__);
            return EXIT_SUCCESS;
        }
        

        /**
         * @brief 读取轨迹文件
         * @param trajectory_file 轨迹文件路径
         * @param poses 存储读取的位姿
         * @return 返回状态码
         */
        int32_t read_trajectory_file(const std::string& trajectory_file, std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>& trajectory) {
            // 读取轨迹文件
            std::ifstream t_file(trajectory_file);
            if (!t_file) {
                RCLCPP_INFO(this->get_logger(), "无法打开轨迹文件 %s", trajectory_file.c_str());
                return EXIT_FAILURE;
            }
            // 逐行读取数据
            while (!t_file.eof()) {
                double time, tx, ty, tz, qx, qy, qz, qw;
                t_file >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
                Sophus::SE3d p1(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
                trajectory.push_back(p1);
            }
            
            return EXIT_SUCCESS;
        }
    };

}   // namespace Lecture4
}   // namespace VisualSLAM