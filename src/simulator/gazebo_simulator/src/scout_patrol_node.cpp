#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
// #include "gazebo_msgs/msg/model_state.hpp"
// #include "gazebo_msgs/msg/model_states.hpp"
#include "gazebo_msgs/srv/get_model_list.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "gazebo_msgs/msg/entity_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include <utility>
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <mutex>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <atomic>

using namespace std::chrono_literals;

struct SensorMapping {
    std::string name;
    std::string original_topic;
    std::string transformed_topic;
    std::string frame_id; // 变换后点云的目标 frame
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
};

class AsyncClient: public rclcpp::Node {
public:
    AsyncClient(): Node("async_client_node") {}
    ~AsyncClient() {
        this->reset_clients();
    }

    void init_clients() {
        if (!get_entity_state_client_) {
            RCLCPP_INFO(this->get_logger(), "初始化 Gazebo 实体状态服务客户端");
            get_entity_state_client_ = this->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state");
        }
        if (!set_entity_state_client_) {
            RCLCPP_INFO(this->get_logger(), "初始化 Gazebo 实体状态服务客户端");
            set_entity_state_client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");
        }

        initialized_ = true;
    }

    void reset_clients() {
        if (get_entity_state_client_) {
            RCLCPP_INFO(this->get_logger(), "重置 Gazebo 实体状态服务客户端");
            get_entity_state_client_.reset();
        }
        if (set_entity_state_client_) {
            RCLCPP_INFO(this->get_logger(), "重置 Gazebo 实体状态服务客户端");
            set_entity_state_client_.reset();
        }
        initialized_ = false;
    }

    bool is_initialized() const {
        return initialized_;
    }

    // 设置gazebo实体状态
    bool setGazeboEntityState(gazebo_msgs::srv::SetEntityState::Request::SharedPtr &request) {
        if (set_entity_state_client_ && set_entity_state_client_->service_is_ready()) {
            auto result_future = set_entity_state_client_->async_send_request(request);
            // 异步等待
            rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);
            auto response = result_future.get();
            return response->success;
        }
        return false;
    }

    // 获取gazebo实体状态，作为当前位姿
    bool getGazeboEntityState(gazebo_msgs::srv::GetEntityState::Request::SharedPtr request, gazebo_msgs::srv::GetEntityState::Response &response) {
        if (get_entity_state_client_ && get_entity_state_client_->service_is_ready()) {
            auto result_future = get_entity_state_client_->async_send_request(request);
            // 异步等待
            rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);
            response = *result_future.get();
            return response.success;
        }
        return false;
    }


private:
    bool initialized_ = false;

    // 与 Gazebo 交互的client
    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr get_entity_state_client_;       // 获取实体状态服务客户端
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_entity_state_client_;       // 设置实体状态服务客户端
};

class ScoutPatrolNode : public rclcpp::Node {
public:
ScoutPatrolNode(): Node("scout_patrol_node") {
    // 获取仿真包的share目录
    try {
        pkg_share_ = ament_index_cpp::get_package_share_directory("gazebo_simulator");
    } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "无法确定包的 share 目录: %s", e.what());
        pkg_share_ = ".";
    }

    // 获取配置文件参数、巡逻速度、位姿发布频率
    this->declare_parameter<std::string>("config_file", pkg_share_ + "/config/scout_params.yaml");
    this->declare_parameter<double>("patrol_speed", 0.5); // 巡逻速度 (m/s)
    this->declare_parameter<double>("pose_publish_rate", 20.0); // 位姿发布频率 (Hz)
    this->declare_parameter<bool>("save_pointclouds", false);
    this->declare_parameter<std::string>("save_directory", pkg_share_ + "/pointclouds");

    config_file_ = this->get_parameter("config_file").as_string();
    speed_ = this->get_parameter("patrol_speed").as_double();
    pose_rate_ = this->get_parameter("pose_publish_rate").as_double();
    save_pointclouds_ = this->get_parameter("save_pointclouds").as_bool();
    save_directory_ = this->get_parameter("save_directory").as_string();

    // 加载配置文件
    if (!load_config(config_file_)) {
        RCLCPP_ERROR(this->get_logger(), "加载配置失败: %s", config_file_.c_str());
    }
    async_client_ = std::make_shared<AsyncClient>();

    if (save_pointclouds_) {
        try {
            std::filesystem::path p(save_directory_);
            std::filesystem::create_directories(p);
            RCLCPP_INFO(this->get_logger(), "点云保存已启用，目录: %s", save_directory_.c_str());
        } catch (const std::exception &ex) {
            RCLCPP_WARN(this->get_logger(), "无法创建点云保存目录 '%s': %s", save_directory_.c_str(), ex.what());
            save_pointclouds_ = false;
        }
    }

    // 创建服务客户端
    get_model_list_client_ = this->create_client<gazebo_msgs::srv::GetModelList>("/get_model_list");

    // 循环检查机器人模型是否存在的定时器
    check_model_timer_ = this->create_wall_timer(1s, std::bind(&ScoutPatrolNode::checkModelTimerCallback, this));

    // 不在构造时直接启动巡逻，改为依赖模型列表检查决定何时启动

    RCLCPP_INFO(this->get_logger(), "ScoutPatrolNode 已启动; config=%s; 机器人=%s; 路径点=%zu", config_file_.c_str(), robot_name_.c_str(), patrol_points_.size());
}

~ScoutPatrolNode() {
    RCLCPP_INFO(this->get_logger(), "ScoutPatrolNode 正在关闭...");
    stopPatrol();
}

private:
// 加载YAML配置文件
bool load_config(const std::string &path) {
    try {
        YAML::Node doc = YAML::LoadFile(path);
        YAML::Node params = YAML::Node();
        if (doc["pointcloud_transformer"] && doc["pointcloud_transformer"]["ros__parameters"]) {
            // 配置中可能包含名为 pointcloud_transformer 的块，并在其中使用 ros__parameters
            params = doc["pointcloud_transformer"]["ros__parameters"];
        } else if (doc["pointcloud_transformer"]) {
            params = doc["pointcloud_transformer"];
        } else {
            params = doc;
        }

        if (params["robot_name"]) robot_name_ = params["robot_name"].as<std::string>();
        if (params["patrol_speed"]) speed_ = params["patrol_speed"].as<double>();
        if (params["pose_publish_rate"]) pose_rate_ = params["pose_publish_rate"].as<double>();
        if (params["save_pointclouds"]) save_pointclouds_ = params["save_pointclouds"].as<bool>();
        if (params["save_directory"]) save_directory_ = params["save_directory"].as<std::string>();

        RCLCPP_INFO(this->get_logger(), "加载配置: 机器人名称=%s", robot_name_.c_str());
        if (params["start_pose"]) {
            auto sp = params["start_pose"];
            if (sp.IsSequence() && sp.size() >= 7) {
                start_pose_.position.x = sp[0].as<double>();
                start_pose_.position.y = sp[1].as<double>();
                start_pose_.position.z = sp[2].as<double>();
                start_pose_.orientation.x = sp[3].as<double>();
                start_pose_.orientation.y = sp[4].as<double>();
                start_pose_.orientation.z = sp[5].as<double>();
                start_pose_.orientation.w = sp[6].as<double>();
            }
        }
        RCLCPP_INFO(this->get_logger(), "加载配置: 起始位置=(%.2f, %.2f, %.2f)", start_pose_.position.x, start_pose_.position.y, start_pose_.position.z);

        if (params["patrol_route"]) {
            auto pr = params["patrol_route"];
            if (pr["is_loop"]) is_loop_ = pr["is_loop"].as<bool>();
                if (pr["points"]) {
                auto pts = pr["points"];
                for (std::size_t i = 0; i < pts.size(); ++i) {
                    const YAML::Node &p = pts[i];
                    if (p.IsSequence() && p.size() >= 7) {
                        geometry_msgs::msg::Pose pose;
                        pose.position.x = p[0].as<double>();
                        pose.position.y = p[1].as<double>();
                        pose.position.z = p[2].as<double>();
                        pose.orientation.x = p[3].as<double>();
                        pose.orientation.y = p[4].as<double>();
                        pose.orientation.z = p[5].as<double>();
                        pose.orientation.w = p[6].as<double>();
                        patrol_points_.push_back(pose);
                    }
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "加载配置: 巡逻机器人%s巡逻路径关键点数量=%zu", robot_name_.c_str(), patrol_points_.size());

        // 传感器映射
        if (params["sensors"]) {
            auto ss = params["sensors"];
            for (std::size_t i = 0; i < ss.size(); ++i) {
                const YAML::Node &si = ss[i];
                SensorMapping s;
                s.name = si["name"] ? si["name"].as<std::string>() : ("sensor_" + std::to_string(i));
                s.original_topic = si["original_topic"] ? si["original_topic"].as<std::string>() : "";
                s.transformed_topic = si["transformed_topic"] ? si["transformed_topic"].as<std::string>() : (s.name + std::string("/transformed"));
                s.frame_id = si["frame_id"] ? si["frame_id"].as<std::string>() : "base_link";
                sensors_.push_back(s);
            }
        }
        RCLCPP_INFO(this->get_logger(), "加载配置: 配置的传感器数量=%zu", sensors_.size());

        return true;
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(this->get_logger(), "加载YAML文件解析失败 %s: %s", path.c_str(), ex.what());
        return false;
    }
}

void chooseNearestWaypoint() {
    // 选择最近的巡逻点。此函数假定调用者已适当管理并发（例如在 onModelStates 之后调用）。
    if (patrol_points_.empty()) return;
    double best_d = 1e9;
    size_t best_idx = 0;
    // 不在此处加锁，调用者需保证一致性
    for (size_t i = 0; i < patrol_points_.size(); ++i) {
        double dx = patrol_points_[i].position.x - current_pose_.position.x;
        double dy = patrol_points_[i].position.y - current_pose_.position.y;
        double d = std::hypot(dx, dy);
        if (d < best_d) { best_d = d; best_idx = i; }
    }
    current_waypoint_idx_ = best_idx;
    RCLCPP_INFO(this->get_logger(), "开始巡逻，起始目标点 %zu，距离=%.3f 米", current_waypoint_idx_, best_d);
}

void movementStep() {
    step_count++;
    getGazeboEntityState("移动迭代开始, ");
    // RCLCPP_INFO(this->get_logger(), "执行移动步进");
    // 移动步进函数：读取共享状态到本地副本以降低锁持有时间，
    // 并确保 publish 在不持有锁的情况下进行。
    if (patrol_points_.empty()) return;

    geometry_msgs::msg::Pose local_pose = current_pose_;
    geometry_msgs::msg::Pose target_pose = patrol_points_[current_waypoint_idx_];

    // 按照速度向目标点移动
    double dist = std::hypot(target_pose.position.x - local_pose.position.x, target_pose.position.y - local_pose.position.y);
    double step = speed_ / pose_rate_;
    if (dist < step) {
        // 已到达当前目标点，选择下一个目标点
        if (is_loop_) {
            current_waypoint_idx_ = (current_waypoint_idx_ + 1) % patrol_points_.size();
        } else {
            if (current_waypoint_idx_ + 1 < patrol_points_.size()) {
                current_waypoint_idx_ += 1;
            } else {
                RCLCPP_INFO(this->get_logger(), "已到达巡逻路径终点，停止移动");
                stopPatrol();
                return;
            }
        }
    }
    target_pose = patrol_points_[current_waypoint_idx_];
    dist = std::hypot(target_pose.position.x - local_pose.position.x, target_pose.position.y - local_pose.position.y);
    double ratio = step / dist;
    geometry_msgs::msg::Pose move_pose;
    move_pose.position.x = local_pose.position.x + (target_pose.position.x - local_pose.position.x) * ratio;
    move_pose.position.y = local_pose.position.y + (target_pose.position.y - local_pose.position.y) * ratio;
    move_pose.position.z = local_pose.position.z + (target_pose.position.z - local_pose.position.z) * ratio;
    move_pose.orientation = target_pose.orientation;
    setGazeboEntityState(move_pose);
    getGazeboEntityState("移动迭代结束, ");
}

void onPointCloud(const std::string &sensor_name, const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "第 %d 次循环, 收到传感器 '%s' 的点云消息，点数=%zu", step_count, sensor_name.c_str(), msg->data.size() / msg->point_step);
    // 查找传感器映射
    auto it = std::find_if(sensors_.begin(), sensors_.end(), [&](const SensorMapping &s){ return s.name == sensor_name; });
    if (it == sensors_.end()) return;
    auto &smap = *it;
    // 使用机器人位姿将点云从机器人局部坐标变换到世界坐标（假设输入点云为机器人局部坐标）
    sensor_msgs::msg::PointCloud2 out = *msg;
    // 确保已有机器人位姿
    if (!have_pose_) {
    // 无位姿可用：直接发布原始点云，但标记为配置的目标 frame
      return;
    }

    // 遍历点云的 x,y,z 分量
    try {
        sensor_msgs::PointCloud2Iterator<float> in_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> in_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> in_z(*msg, "z");
        sensor_msgs::PointCloud2Iterator<float> out_x(out, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(out, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(out, "z");

        // 读取共享 current_pose_ 的本地副本以保证线程安全
        geometry_msgs::msg::Pose local_pose;
        getGazeboEntityState("点云处理前, ");
        {
            std::lock_guard<std::mutex> lk(mutex_);
            local_pose = current_entity_state_.state.pose;
        }
        // 计算点云时间戳与机器人位姿时间戳的差异（假设位姿时间戳为最新）
        rclcpp::Time pc_time = rclcpp::Time(msg->header.stamp);
        rclcpp::Time pose_time = rclcpp::Time(current_entity_state_.header.stamp);
        double time_diff = (pose_time - pc_time).seconds();
        // RCLCPP_INFO(this->get_logger(), "补偿点云时间戳与机器人位姿时间戳的差异: %.5f 秒 ? %.5f 秒", time_diff, 1.0 / pose_rate_);
        if (time_diff > -0.001) {    // 雷达频率周期的一半
            RCLCPP_INFO(this->get_logger(), "补偿点云时间戳与机器人位姿时间戳的差异: %.5f 秒 大于 %.5f 秒", time_diff, -0.001);
            local_pose = last_entity_state_.state.pose;
        }

        tf2::Quaternion q;
        q.setX(local_pose.orientation.x);
        q.setY(local_pose.orientation.y);
        q.setZ(local_pose.orientation.z);
        q.setW(local_pose.orientation.w);
        tf2::Vector3 trans(local_pose.position.x, local_pose.position.y, local_pose.position.z);

        for (; in_x != in_x.end(); ++in_x, ++in_y, ++in_z, ++out_x, ++out_y, ++out_z) {
            tf2::Vector3 p_local(*in_x, *in_y, *in_z);
            // tf2::Vector3 p_world = tf2::quatRotate(q, p_local) + trans;
            tf2::Vector3 p_world = p_local + trans;
            *out_x = p_world.x();
            *out_y = p_world.y();
            *out_z = p_world.z();
        }
        out.header.frame_id = smap.frame_id;
        smap.pub->publish(out);
        // 可选：将变换后的点云保存到本地目录
        if (save_pointclouds_) {
            try {
                savePointCloudToPCD(out, smap.name);
            } catch (const std::exception &ex) {
                RCLCPP_WARN(this->get_logger(), "保存点云失败: %s", ex.what());
            }
        }
        
    } catch (const std::exception &ex) {
        RCLCPP_WARN(this->get_logger(), "点云变换错误: %s", ex.what());
    }
}

// 将 PointCloud2 保存为 ASCII PCD 文件（只保存 x,y,z 字段）
void savePointCloudToPCD(const sensor_msgs::msg::PointCloud2 &cloud, const std::string &sensor_name) {
    // 仅支持包含 x,y,z 的点云
    uint64_t points = static_cast<uint64_t>(cloud.width) * static_cast<uint64_t>(cloud.height);
    if (points == 0) return;

    // 构造文件名：<save_directory>/<sensor>_<sec>_<nsec>_<seq>.pcd
    // 使用 header 时间戳的 sec + nanosec 组成文件名，避免小数点问题
    uint32_t sec = cloud.header.stamp.sec;
    uint32_t nsec = cloud.header.stamp.nanosec;
    uint64_t seq = pc_save_seq_.fetch_add(1);
    std::ostringstream fname;
    fname << save_directory_ << "/" << sensor_name << "_" << sec << "_" << nsec << "_" << step_count << "_" << seq << ".pcd";

    std::ofstream ofs(fname.str());
    if (!ofs.is_open()) {
        throw std::runtime_error("无法打开输出文件: " + fname.str());
    }

    // 写 PCD 头（ASCII, fields x y z）
    ofs << "# .PCD v0.7 - Point Cloud Data file format\n";
    ofs << "VERSION 0.7\n";
    ofs << "FIELDS x y z\n";
    ofs << "SIZE 4 4 4\n";
    ofs << "TYPE F F F\n";
    ofs << "COUNT 1 1 1\n";
    ofs << "WIDTH " << points << "\n";
    ofs << "HEIGHT 1\n";
    ofs << "POINTS " << points << "\n";
    ofs << "DATA ascii\n";

    // 使用迭代器写入点数据
    sensor_msgs::PointCloud2Iterator<float> it_x(const_cast<sensor_msgs::msg::PointCloud2&>(cloud), "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(const_cast<sensor_msgs::msg::PointCloud2&>(cloud), "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(const_cast<sensor_msgs::msg::PointCloud2&>(cloud), "z");

    ofs << std::fixed << std::setprecision(6);
    for (uint64_t i = 0; i < points; ++i, ++it_x, ++it_y, ++it_z) {
        ofs << *it_x << " " << *it_y << " " << *it_z << "\n";
    }

    ofs.close();
    // 序列号已经通过 atomic fetch_add 增加
    RCLCPP_INFO(this->get_logger(), "已保存点云: %s", fname.str().c_str());
}

void checkModelTimerCallback() {
    using GetModelList = gazebo_msgs::srv::GetModelList;

    // 检测客户端是否可用
    rclcpp::Client<GetModelList>::SharedPtr client;
    if (get_model_list_client_ && get_model_list_client_->service_is_ready()) {
        client = get_model_list_client_;
    } else {
        // 如果服务尚不可用，短时间内跳过（非阻塞）
        RCLCPP_WARN(this->get_logger(), "获取模型列表服务不可用 (/get_model_list), 请检测是否安装 ros-humble-gazebo-ros 以及 启动Gazebo");
        stopPatrol();
        return;
    }

    // 发送异步请求；回调会更新 robot_exists_ 状态
    auto req = std::make_shared<GetModelList::Request>();
    auto result_future = client->async_send_request(req,
        [this](rclcpp::Client<GetModelList>::SharedFuture future) {
            try {
                auto res = future.get();
                bool found = false;
                std::string model_list = "|";
                for (const auto &n : res->model_names) {
                    model_list += " " + n + " |";
                    if (n == robot_name_) { found = true; break; }
                }

                bool do_start = false;
                bool do_stop = false;
                {
                    std::lock_guard<std::mutex> lk(mutex_);
                    if (!found && robot_exists_) {
                        do_stop = true;
                    } else if (found && !robot_exists_) {
                        // 在外部 startPatrol 之前，标记 initial_pose 需要重新设置
                        initial_pose_set_ = false;
                        do_start = true;
                    }
                }

                if (do_stop) {
                    RCLCPP_INFO(this->get_logger(), "当前Gazebo模型列表: %s", model_list.c_str());
                    RCLCPP_WARN(this->get_logger(), "巡逻机器人 '%s' 不在Gazebo模型列表中; 将停止巡逻并清理资源", robot_name_.c_str());
                    this->stopPatrol();
                }
                if (do_start) {
                    RCLCPP_INFO(this->get_logger(), "巡逻机器人 '%s' 出现在Gazebo模型列表中; 将在可用时重新初始化位姿并启动巡逻", robot_name_.c_str());
                    this->startPatrol();
                }
            } catch (const std::exception &ex) {
                RCLCPP_WARN(this->get_logger(), "调用模型列表服务失败: %s", ex.what());
            }
        }
    );
}

// 停止当前巡逻
void stopPatrol() {
    RCLCPP_INFO(this->get_logger(), "正在停止巡逻，机器人 '%s'", robot_name_.c_str());
    // 取消并销毁所有运行时对象，彻底清理巡逻相关资源
    if (movement_timer_) {
        RCLCPP_INFO(this->get_logger(), "销毁运动定时器");
        try { movement_timer_->cancel(); } catch (...) {}
        movement_timer_.reset();
    }

    // 销毁传感器相关的发布者/订阅者
    for (auto &s : sensors_) {
        RCLCPP_INFO(this->get_logger(), "销毁传感器 '%s' 的发布者和订阅者", s.name.c_str());
        if (s.sub) { s.sub.reset(); }
        if (s.pub) { s.pub.reset(); }
    }

    // 销毁与 Gazebo 交互相关的发布者/订阅者
    if (async_client_->is_initialized()) {
        RCLCPP_INFO(this->get_logger(), "销毁获取实体状态服务客户端");
        async_client_->reset_clients();
    }

    // 注意: 不销毁 check_model_timer_ 与 get_model_list_client_，
    // 这样节点可以继续监测模型是否重新出现并在需要时重启巡逻。

    // 重置运行时状态（在互斥下写回）
    {
        std::lock_guard<std::mutex> lk(mutex_);
        initial_pose_set_ = false;
        have_pose_ = false;
        robot_exists_ = false;
        current_waypoint_idx_ = 0;
        step_count = 0;
        current_pose_ = geometry_msgs::msg::Pose();
    }

    RCLCPP_INFO(this->get_logger(), "巡逻已完全停止，运行数据已销毁，机器人 '%s'", robot_name_.c_str());
}

// 开始巡逻
void startPatrol() {
    RCLCPP_INFO(this->get_logger(), "正在启动巡逻，机器人 '%s'", robot_name_.c_str());
    // 确保参数获取、修改客户端存在
    if (!async_client_->is_initialized()) {
        async_client_->init_clients();
    }

    // 恢复传感器发布/订阅
    for (auto &s : sensors_) {
        if (!s.pub) {
            RCLCPP_INFO(this->get_logger(), "创建传感器 '%s' 的点云发布者，话题: %s", s.name.c_str(), s.transformed_topic.c_str());
            s.pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(s.transformed_topic, 10);
        }
        if (!s.sub) {
            RCLCPP_INFO(this->get_logger(), "创建传感器 '%s' 的点云订阅者，话题: %s", s.name.c_str(), s.original_topic.c_str());
            s.sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(s.original_topic, 10,
                [this, name = s.name](const sensor_msgs::msg::PointCloud2::SharedPtr msg){ this->onPointCloud(name, msg); });
        }
    }

    // 配置起始位姿
    configureInitialPose();

    // 启动运动定时器
    if (!movement_timer_) {
        RCLCPP_INFO(this->get_logger(), "创建运动定时器，频率=%.2f Hz", pose_rate_);
        auto period = std::chrono::duration<double>(1.0 / pose_rate_);
        movement_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
          std::bind(&ScoutPatrolNode::movementStep, this));
    }

    {
        std::lock_guard<std::mutex> lk(mutex_);
        robot_exists_ = true;
    }
    RCLCPP_INFO(this->get_logger(), "巡逻已启动，机器人 '%s'", robot_name_.c_str());
}
// 配置起始位姿
void configureInitialPose() {
    if (initial_pose_set_) return; // 已设置则跳过

    RCLCPP_INFO(this->get_logger(), "配置机器人 '%s' 的起始位姿", robot_name_.c_str());

    if (!async_client_->is_initialized()) {
        RCLCPP_WARN(this->get_logger(), "设置实体状态服务客户端尚未准备好");
        return;
    }

    // 计算配置文件位姿与巡逻路径中最近点的距离
    double start_x = start_pose_.position.x;
    double start_y = start_pose_.position.y;
    double best_d = 1e9;
    int32_t best_idx = -1;
    for (size_t i = 0; i < patrol_points_.size(); ++i) {
        const auto &pt = patrol_points_[i];
        double dx = pt.position.x - start_x;
        double dy = pt.position.y - start_y;
        double d = std::hypot(dx, dy);
        if (d < best_d) {
            best_d = d;
            best_idx = static_cast<int32_t>(i);
        }
    }
    if (best_d > 1.5) {
        // 将位置直接设置到巡逻点第一个，目标点设置为第二个
        RCLCPP_WARN(this->get_logger(), "起始位姿与巡逻路径点距离过远 (%.2f 米)，将位置设置为第一个巡逻点", best_d);
        setGazeboEntityState(patrol_points_[0]);
        current_waypoint_idx_ = (patrol_points_.size() >= 2) ? 1 : 0;
    } else {
        setGazeboEntityState(start_pose_);
        RCLCPP_INFO(this->get_logger(), "起始位姿与巡逻路径点距离 %.2f 米，选择该点作为目标位置", best_d);
        current_waypoint_idx_ = best_idx;
    }
}
// 设置gazebo实体状态
void setGazeboEntityState(const geometry_msgs::msg::Pose pose) {
    if (async_client_->is_initialized()) {
        auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        gazebo_msgs::msg::EntityState state;
        state.name = robot_name_;
        state.pose = pose;
        state.twist.linear.x = 0.0;
        state.twist.linear.y = 0.0;
        state.twist.linear.z = 0.0;
        state.twist.angular.x = 0.0;
        state.twist.angular.y = 0.0;
        state.twist.angular.z = 0.0;
        state.reference_frame = "world";
        request->state = state;

        if (async_client_->setGazeboEntityState(request)) {
            RCLCPP_INFO(this->get_logger(), "第 %d 次循环, 成功设置 Gazebo 实体 '%s' 位姿为 [%.3f, %.3f, %.3f]", step_count, robot_name_.c_str(), pose.position.x, pose.position.y, pose.position.z);
            {
                std::lock_guard<std::mutex> lk(mutex_);
                initial_pose_set_ = true;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "第 %d 次循环, 设置 Gazebo 实体 '%s' 位姿失败", step_count, robot_name_.c_str());
        }

    } else {
        RCLCPP_WARN(this->get_logger(), "第 %d 次循环, 实体状态服务客户端尚未准备好", step_count);
    }
}

// 获取gazebo实体状态，作为当前位姿
void getGazeboEntityState(std::string readonly_name="") {
    if (async_client_->is_initialized()) {

        auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
        request->name = robot_name_;
        request->reference_frame = "world";

        auto response = gazebo_msgs::srv::GetEntityState::Response();
        if (async_client_->getGazeboEntityState(request, response)) {
            {
                std::lock_guard<std::mutex> lk(mutex_);
                // 计算获取到的位姿与当前位姿的距离
                double dx = response.state.pose.position.x - current_pose_.position.x;
                double dy = response.state.pose.position.y - current_pose_.position.y;
                // double dz = response.state.pose.position.z - current_pose_.position.z;
                double distance = std::hypot(dx, dy);
                // RCLCPP_INFO(this->get_logger(), "第 %d 次循环, %s获取到的位姿位置=(%.3f, %.3f, %.3f)", step_count, readonly_name.c_str(), response.state.pose.position.x, response.state.pose.position.y, response.state.pose.position.z);
                if (distance > 5.0) {
                    RCLCPP_WARN(this->get_logger(), "第 %d 次循环, %s获取到的位姿与当前位姿偏差过大 (%.2f 米)，可能存在问题", step_count, readonly_name.c_str(), distance);
                    return;
                } else if (distance < 0.001) {
                    // RCLCPP_INFO(this->get_logger(), "第 %d 次循环, %s获取到的位姿与当前位姿无显著变化 (%.4f 米)", step_count, readonly_name.c_str(), distance);
                    return;
                }

                last_entity_state_ = current_entity_state_;
                current_entity_state_ = response;
                current_pose_ = response.state.pose;
                have_pose_ = true;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "第 %d 次循环, %s获取 Gazebo 实体 '%s' 位姿失败", step_count, readonly_name.c_str(), robot_name_.c_str());
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "第 %d 次循环, %s实体状态服务客户端尚未准备好", step_count, readonly_name.c_str());
    }
}

private:
    int step_count = 0;
    // 配置参数
    std::string pkg_share_;
    std::string config_file_;
    std::string robot_name_ = "scout_bot";
    // 机器人当前、起始位姿（仅 pose 部分）
    geometry_msgs::msg::Pose start_pose_;
    geometry_msgs::msg::Pose current_pose_;
    gazebo_msgs::srv::GetEntityState::Response last_entity_state_;
    gazebo_msgs::srv::GetEntityState::Response current_entity_state_;
    // 巡逻路径
    bool is_loop_ = true;                                       // 是否循环巡逻
    size_t current_waypoint_idx_ = 0;                           // 当前目标巡逻点索引
    std::vector<geometry_msgs::msg::Pose> patrol_points_;       // 巡逻路径点列表
    // 运行时状态
    bool have_pose_ = false;
    bool initial_pose_set_ = false;
    bool robot_exists_ = false;
    // 巡逻参数
    double speed_ = 0.5;
    double pose_rate_ = 20.0;
    // 与 Gazebo 交互的客户端
    std::shared_ptr<AsyncClient> async_client_ = nullptr;
    // 时钟
    rclcpp::TimerBase::SharedPtr check_model_timer_;    // 定时检查模型状态的定时器
    rclcpp::TimerBase::SharedPtr movement_timer_;       // 定时更新巡逻位置的定时器
    // 服务客户端：用于查询 Gazebo 模型列表
    rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedPtr get_model_list_client_;

    std::vector<SensorMapping> sensors_;
    // 保护运行时共享数据的互斥锁
    std::mutex mutex_;

    // 点云保存相关
    bool save_pointclouds_ = false;
    std::string save_directory_;
    std::atomic<uint64_t> pc_save_seq_{0};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScoutPatrolNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
