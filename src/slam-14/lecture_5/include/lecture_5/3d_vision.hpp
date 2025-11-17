#pragma once
#include "interface/common/test_interface.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <vector>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>

namespace VisualSLAM {
namespace Lecture5 {
    class Vision3DTest: public FunctionTest::FunctionTestInterface {
        using Vector6d = Eigen::Matrix<double, 6, 1>;
    public:
        Vision3DTest() : FunctionTest::FunctionTestInterface("vision_3d_test_node") {
            // 添加测试函数
            // add_test_function(std::bind(&Vision3DTest::stereo_vision_test, this));
            add_test_function(std::bind(&Vision3DTest::rgbd_vision_test, this));
        }

    private:
        /**
         * @brief 双目相机
         */
        int32_t stereo_vision_test() {
            RCLCPP_INFO(logger_, "%s::%s 双目相机测试开始",demangle(typeid(*this).name()).c_str(),  __func__);
            // 加载左右两侧图像
            cv::Mat left_image = cv::imread(self_package_path_ + "/config/left.png", cv::ImreadModes::IMREAD_GRAYSCALE);
            cv::Mat right_image = cv::imread(self_package_path_ + "/config/right.png", cv::ImreadModes::IMREAD_GRAYSCALE);
            if (left_image.empty() || right_image.empty()) {
                RCLCPP_INFO(logger_, "图像加载失败");
            }
            // 左右图像展示
            cv::namedWindow("Left Image", cv::WINDOW_NORMAL);
            cv::moveWindow("Left Image", 0, 0);
            cv::imshow("Left Image", left_image);
            cv::namedWindow("Right Image", cv::WINDOW_NORMAL);
            cv::moveWindow("Right Image", 0, left_image.rows);
            cv::imshow("Right Image", right_image);
            cv::waitKey(1000);

            // 双目相机标定参数
            // 内参
            double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
            // 基线长度
            double baseline = 0.573;

            // OpenCV计算视差图
            cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(0, 96, 9, 8*3*9*9, 32*3*9*9, 1, 0, 63, 10, 100, 32);
            cv::Mat disparity, disparity_float;
            stereo->compute(left_image, right_image, disparity);
            disparity.convertTo(disparity_float, CV_32F, 1.0 / 16.0f);

            // 生成点云
            std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> point_cloud;
            for (int v = 0; v < left_image.rows; v++) {
                for (int u = 0; u < left_image.cols; u++) {
                    float disp = disparity_float.at<float>(v, u);
                    if (disp <= 0.0) continue; // 无效视差值跳过

                    // 计算3D坐标
                    float Z = fx * baseline / disp;
                    float X = (u - cx) * Z / fx;
                    float Y = (v - cy) * Z / fy;

                    // 获取颜色信息
                    std::uint8_t intensity = left_image.at<std::uint8_t>(v, u);
                    point_cloud.emplace_back(Vector6d(X, Y, Z, intensity, intensity, intensity));
                }
            }

            // 视差图展示
            cv::Mat disp8;
            disparity_float.convertTo(disp8, CV_8U, 255 / 96.0);
            cv::namedWindow("Disparity", cv::WINDOW_NORMAL);
            cv::moveWindow("Disparity", 0, left_image.rows * 2);
            cv::imshow("Disparity", disp8);
            cv::waitKey(1000);

            // 销毁窗口
            cv::destroyAllWindows();

            // 显示点云
            showPointCloud(point_cloud);
            RCLCPP_INFO(logger_, "%s::%s 双目相机测试结束",demangle(typeid(*this).name()).c_str(),  __func__);
            return EXIT_SUCCESS;
        }

        /**
         * @brief RGB-D相机
         */
        int32_t rgbd_vision_test() {
            RCLCPP_INFO(logger_, "%s::%s RGB-D相机测试开始",demangle(typeid(*this).name()).c_str(),  __func__);
            // 读取相机位姿参数
            std::vector<Eigen::VectorXd> poses;
            std::ifstream pose_file(self_package_path_ + "/config/pose.txt");
            if (!pose_file.is_open()) {
                RCLCPP_ERROR(logger_, "无法打开位姿文件");
                return EXIT_FAILURE;
            }
            std::string line;
            while (std::getline(pose_file, line)) {
                std::istringstream iss(line);
                Eigen::VectorXd pose(7);            // x, y, z, qx, qy, qz, qw
                for (int i = 0; i < 7; i++) {
                    iss >> pose(i);
                }
                poses.push_back(pose);
            }
            pose_file.close();

            // 读取RGB图像和深度图像
            std::vector<cv::Mat> rgb_images, depth_images;
            for (int i = 1; i <= poses.size(); i++) {
                cv::Mat rgb = cv::imread(self_package_path_ + "/config/color/" + std::to_string(i) + ".png", cv::ImreadModes::IMREAD_COLOR);
                cv::Mat depth = cv::imread(self_package_path_ + "/config/depth/" + std::to_string(i) + ".pgm", cv::ImreadModes::IMREAD_UNCHANGED);
                if (rgb.empty() || depth.empty()) {
                    RCLCPP_ERROR(logger_, "图像%d加载失败", i);
                    return EXIT_FAILURE;
                }
                rgb_images.push_back(rgb);
                depth_images.push_back(depth);
            }

            // 相机内参
            double fx = 518.0, fy = 519.0, cx = 325.55, cy = 253.5;
            double depth_scale = 1000.0; // 深度图像缩放因子

            // 预留点云空间
            std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> point_cloud;
            point_cloud.reserve(1000000);

            // 遍历每一帧图像
            for (size_t i = 0; i < rgb_images.size(); i++) {
                const cv::Mat &rgb = rgb_images[i];
                const cv::Mat &depth = depth_images[i];
                const Sophus::SE3d pose(
                    Eigen::Quaterniond(poses[i](6), poses[i](3), poses[i](4), poses[i](5)),
                    Eigen::Vector3d(poses[i](0), poses[i](1), poses[i](2))
                );

                // 生成点云
                for (int v = 0; v < depth.rows; v++) {
                    for (int u = 0; u < depth.cols; u++) {
                        uint16_t d = depth.ptr<uint16_t>(v)[u];
                        if (d == 0) continue;
                        double z = double(d) / depth_scale;
                        double x = (u - cx) * z / fx;
                        double y = (v - cy) * z / fy;

                        // 将点云坐标转换到世界坐标系
                        Eigen::Vector3d point_world = pose * Eigen::Vector3d(x, y, z);
                        Eigen::Vector3d color(rgb.ptr<uint8_t>(v)[u * 3], rgb.ptr<uint8_t>(v)[u * 3 + 1], rgb.ptr<uint8_t>(v)[u * 3 + 2]);
                        point_cloud.push_back(Vector6d(point_world[0], point_world[1], point_world[2], color[0], color[1], color[2]));
                    }
                }
            }
            RCLCPP_INFO(logger_, "点云生成完毕, 点云数量: %zu", point_cloud.size());

            // 显示点云
            showPointCloud(point_cloud);

            RCLCPP_INFO(logger_, "%s::%s RGB-D相机测试结束",demangle(typeid(*this).name()).c_str(),  __func__);
            return EXIT_SUCCESS;
        }


        void showPointCloud(const std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud) {

            if (pointcloud.empty()) {
                RCLCPP_INFO(logger_, "点云数据为空，无法显示！");
                return;
            }

            pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
                pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
            );

            pangolin::View &d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));

            while (pangolin::ShouldQuit() == false) {
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                d_cam.Activate(s_cam);
                glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

                glPointSize(2);
                glBegin(GL_POINTS);
                for (auto &p: pointcloud) {
                    glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
                    glVertex3d(p[0], p[1], p[2]);
                }
                glEnd();
                pangolin::FinishFrame();
                usleep(5000);   // sleep 5 ms
            }
            return;
        }
    };
}   // namespace Lecture5
}   // namespace VisualSLAM