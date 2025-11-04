#include <memory>
#include <string>
#include <vector>
#include <map>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <laser_geometry/laser_geometry.hpp>
#include <yaml-cpp/yaml.h>
#include <any>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

struct MappingEntry {
    std::string original_topic;
    std::string transformed_topic;
    std::string frame_id; // target frame
    bool active = false;
    std::any subscription_any; // keep subscription shared_ptr
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
    std::string type_name;
    // optional static transform (applied from message frame -> frame_id)
    bool has_static_transform = false;
    geometry_msgs::msg::Transform static_transform;
};

class PointCloudTransformer : public rclcpp::Node {
public:
    PointCloudTransformer(): Node("pointcloud_transformer") {
        // 获取包安装路径
        try {
            RCLCPP_INFO(this->get_logger(), "尝试获取ROS2包: agv_process_planning的share文件夹安装路径");
            this->node_path_ = ament_index_cpp::get_package_share_directory("gazebo_simulator");
        } catch (const std::exception &e) {     // safe: 兼容所有 ROS/ament 版本
            RCLCPP_ERROR(this->get_logger(), "获取ROS2包路径失败, 原因: %s", e.what());
        }

        this->declare_parameter<std::string>("config_file", "/config/topic_transform.yaml");
        this->declare_parameter<double>("check_period_sec", 1.0);

        config_file_ =  this->node_path_ + this->get_parameter("config_file").as_string();
        check_period_ = this->get_parameter("check_period_sec").as_double();

        if (!load_config(config_file_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load config file: %s", config_file_.c_str());
            // still continue: node will have empty mappings
        }

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // laser projector
        // projector_ is default-constructed

        // periodic timer to check topics and create typed subscriptions when available
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(check_period_)),
            std::bind(&PointCloudTransformer::check_and_setup_subscriptions, this)
        );

        RCLCPP_INFO(this->get_logger(), "PointCloudTransformer started; config_file=%s; check_period=%.2f", config_file_.c_str(), check_period_);
    }

private:
    bool load_config(const std::string &path) {
        try {
            YAML::Node doc = YAML::LoadFile(path);
            YAML::Node seq = YAML::Node();
            // support structure: pointcloud_transformer: { ros__parameters: [ - original_topic: ... ] }
            if (doc["pointcloud_transformer"] && doc["pointcloud_transformer"]["ros__parameters"])
            {
                seq = doc["pointcloud_transformer"]["ros__parameters"];
            } else if (doc["mappings"]) {
                seq = doc["mappings"];
            } else if (doc.IsSequence()) {
                seq = doc;
            } else {
                RCLCPP_WARN(this->get_logger(), "Config file has no recognized mappings node");
                return false;
            }

            for (std::size_t i = 0; i < seq.size(); ++i) {
                YAML::Node item = seq[i];
                if (!item["original_topic"] || !item["transformed_topic"] || !item["frame_id"]) {
                    RCLCPP_WARN(this->get_logger(), "Skipping malformed mapping entry at index %zu", i);
                    continue;
                }
                MappingEntry e;
                e.original_topic = item["original_topic"].as<std::string>();
                e.transformed_topic = item["transformed_topic"].as<std::string>();
                e.frame_id = item["frame_id"].as<std::string>();
                e.active = false;
                // optional static transform
                if (item["tf"]) {
                    try {
                        YAML::Node tnode = item["tf"];
                        e.has_static_transform = true;
                        e.static_transform.translation.x = tnode[0].as<double>();
                        e.static_transform.translation.y = tnode[1].as<double>();
                        e.static_transform.translation.z = tnode[2].as<double>();
                        e.static_transform.rotation.x = tnode[3].as<double>();
                        e.static_transform.rotation.y = tnode[4].as<double>();
                        e.static_transform.rotation.z = tnode[5].as<double>();
                        e.static_transform.rotation.w = tnode[6].as<double>();
                        RCLCPP_INFO(this->get_logger(), "Mapping %s has static transform tx=%.3f ty=%.3f tz=%.3f q=(%.3f,%.3f,%.3f,%.3f)",
                                    e.original_topic.c_str(), e.static_transform.translation.x, e.static_transform.translation.y, e.static_transform.translation.z,
                                    e.static_transform.rotation.x, e.static_transform.rotation.y, e.static_transform.rotation.z, e.static_transform.rotation.w);
                    } catch (const std::exception &ex) {
                        RCLCPP_WARN(this->get_logger(), "Failed to parse transform for mapping %s: %s", e.original_topic.c_str(), ex.what());
                    }
                }
                mappings_.push_back(std::move(e));
                RCLCPP_INFO(this->get_logger(), "Loaded mapping: %s -> %s (to frame %s)", mappings_.back().original_topic.c_str(), mappings_.back().transformed_topic.c_str(), mappings_.back().frame_id.c_str());
            }
            return true;
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Exception loading config: %s", ex.what());
            return false;
        }
    }

    void check_and_setup_subscriptions() {
        // get current topics and types
        auto topics_and_types = this->get_topic_names_and_types();

        for (size_t i = 0; i < mappings_.size(); ++i) {
            auto &m = mappings_[i];
            if (m.active) continue;
            auto it = topics_and_types.find(m.original_topic);
            if (it == topics_and_types.end()) {
                // not yet present
                continue;
            }
            const auto &types = it->second;
            if (types.empty()) continue;
            // choose first type
            m.type_name = types[0];

            rclcpp::QoS qos( rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default) );
            qos.keep_last(10);

            if (m.type_name == "sensor_msgs/msg/PointCloud2") {
                // create publisher
                m.pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m.transformed_topic, qos);
                // subscription
                auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    m.original_topic, qos,
                    [this, i](const sensor_msgs::msg::PointCloud2::SharedPtr msg){ this->handle_pc2(i, msg); }
                );
                m.subscription_any = sub;
                m.active = true;
                RCLCPP_INFO(this->get_logger(), "Activated PointCloud2 subscription for %s", m.original_topic.c_str());
            } else if (m.type_name == "sensor_msgs/msg/LaserScan") {
                // create publisher (we will publish PointCloud2)
                m.pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m.transformed_topic, qos);
                auto sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    m.original_topic, qos,
                    [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg){ this->handle_laserscan(i, msg); }
                );
                m.subscription_any = sub;
                m.active = true;
                RCLCPP_INFO(this->get_logger(), "Activated LaserScan subscription for %s", m.original_topic.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Topic %s has unsupported type %s", m.original_topic.c_str(), m.type_name.c_str());
                // mark active to avoid repeated warnings? We'll mark active=false so if type changes maybe later
            }
        }
    }

    void handle_pc2(size_t idx, const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (idx >= mappings_.size()) return;
        auto &m = mappings_[idx];
        if (!m.pub) return;
        if (msg->header.frame_id.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Message on %s missing header.frame_id", m.original_topic.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Transforming PointCloud2 %ld from %s to %s", msg->data.size(), m.original_topic.c_str(), m.transformed_topic.c_str());
                try {
                        if (m.has_static_transform) {
                                geometry_msgs::msg::TransformStamped t;
                                t.header.stamp = msg->header.stamp;
                                t.header.frame_id = m.frame_id; // target frame
                                t.child_frame_id = msg->header.frame_id; // source frame
                                t.transform = m.static_transform;
                                sensor_msgs::msg::PointCloud2 out;
                                tf2::doTransform(*msg, out, t);
                                m.pub->publish(out);
                        } else {
                                auto t = tf_buffer_->lookupTransform(m.frame_id, msg->header.frame_id, tf2::TimePointZero);
                                sensor_msgs::msg::PointCloud2 out;
                                tf2::doTransform(*msg, out, t);
                                m.pub->publish(out);
                        }
                } catch (const std::exception &ex) {
                        RCLCPP_DEBUG(this->get_logger(), "TF lookup/transform failed for %s -> %s : %s", msg->header.frame_id.c_str(), m.frame_id.c_str(), ex.what());
                }
    }

    void handle_laserscan(size_t idx, const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (idx >= mappings_.size()) return;
        auto &m = mappings_[idx];
        if (!m.pub) return;
        if (msg->header.frame_id.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Message on %s missing header.frame_id", m.original_topic.c_str());
            return;
        }
        sensor_msgs::msg::PointCloud2 cloud;
        try {
            projector_.projectLaser(*msg, cloud);
        } catch (const std::exception &ex) {
            RCLCPP_WARN(this->get_logger(), "Laser projection failed for %s: %s", m.original_topic.c_str(), ex.what());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Transforming PointCloud2 %ld from %s to %s", cloud.data.size(), m.original_topic.c_str(), m.transformed_topic.c_str());
                try {
                        if (m.has_static_transform) {
                                geometry_msgs::msg::TransformStamped t;
                                t.header.stamp = cloud.header.stamp;
                                t.header.frame_id = m.frame_id; // target frame
                                t.child_frame_id = cloud.header.frame_id; // source frame
                                t.transform = m.static_transform;
                                sensor_msgs::msg::PointCloud2 out;
                                tf2::doTransform(cloud, out, t);
                                m.pub->publish(out);
                        } else {
                                auto t = tf_buffer_->lookupTransform(m.frame_id, cloud.header.frame_id, tf2::TimePointZero);
                                sensor_msgs::msg::PointCloud2 out;
                                tf2::doTransform(cloud, out, t);
                                m.pub->publish(out);
                        }
                } catch (const std::exception &ex) {
                        RCLCPP_DEBUG(this->get_logger(), "TF lookup/transform failed for projected cloud %s -> %s : %s", cloud.header.frame_id.c_str(), m.frame_id.c_str(), ex.what());
                }
    }

private:
    std::string node_path_;
    std::string config_file_;
    double check_period_ = 1.0;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<MappingEntry> mappings_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    laser_geometry::LaserProjection projector_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
