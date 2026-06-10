#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/message_memory_strategy.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// Known message types we can extract a header timestamp from
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <rclcpp/serialization.hpp>

// Standard diagnostics + trigger interfaces
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <exception> // Explicitly include for std::exception
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>
#include <yaml-cpp/yaml.h>

// For dynamic message introspection
#include <rosidl_runtime_cpp/message_type_support_decl.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/visibility_control.h>

using namespace std::chrono_literals;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using KeyValue = diagnostic_msgs::msg::KeyValue;

// Structure to hold QoS configuration
struct QoSConfig
{
    size_t history_depth = 10;
    std::string reliability = "reliable";  // "reliable" or "best_effort"
    std::string durability = "volatile";   // "volatile" or "transient_local"
    
    QoSConfig() = default;
};

// Structure to hold topic information
struct TopicInfo
{
    std::string topic_name;
    std::string message_type;
    int message_count = 0;
    rclcpp::Time first_timestamp;
    rclcpp::Time last_timestamp;
    rclcpp::Time first_received_walltime;  // when the first message arrived
    rclcpp::Time last_received_walltime;   // when the most recent message arrived
    bool received_message_since_start = false;
    std::shared_ptr<rclcpp::GenericSubscription> subscription;
    QoSConfig qos_config;  // Store QoS config for debugging and future features

    // Constructor to properly initialize all members and avoid warnings
    TopicInfo(std::string name, std::string type)
        : topic_name(std::move(name)), message_type(std::move(type)), message_count(0),
          received_message_since_start(false), subscription(nullptr)
    {
    }
};


class TopicMonitor : public rclcpp::Node
{
  public:
    TopicMonitor() : Node("topic_monitor")
    {
        RCLCPP_INFO(this->get_logger(), "In topic_monitor constructor");

        this->declare_parameter<std::string>("topics_file", "config/topics.yaml");
        topics_file_ = this->get_parameter("topics_file").as_string();
        this->declare_parameter<bool>("relative_path", true);
        relative_path_ = this->get_parameter("relative_path").as_bool();
        this->declare_parameter<double>("sync_threshold_warn_seconds", 0.1);
        SYNC_THRESHOLD_WARN_SECONDS = this->get_parameter("sync_threshold_warn_seconds").as_double();
        this->declare_parameter<double>("sync_threshold_error_seconds", 1.0);
        SYNC_THRESHOLD_ERROR_SECONDS = this->get_parameter("sync_threshold_error_seconds").as_double();
        // How long without a new message before a topic is considered "stale".
        this->declare_parameter<double>("stale_timeout_seconds", 2.0);
        STALE_TIMEOUT_SECONDS = this->get_parameter("stale_timeout_seconds").as_double();
        // How often the DiagnosticArray is published on /diagnostics.
        this->declare_parameter<double>("publish_period_seconds", 1.0);
        double publish_period = this->get_parameter("publish_period_seconds").as_double();
        // Prefix prepended to each DiagnosticStatus.name so an aggregator can group them.
        this->declare_parameter<std::string>("diagnostic_name_prefix", "timesync");
        diagnostic_name_prefix_ = this->get_parameter("diagnostic_name_prefix").as_string();

        load_topics_from_file();

        // Continuously publish diagnostics on the conventional global /diagnostics
        // topic (absolute name, so it ignores this node's namespace). This is what
        // rqt_robot_monitor / diagnostic_aggregator consume.
        diagnostics_pub_ = this->create_publisher<DiagnosticArray>("/diagnostics", 10);
        diag_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(publish_period),
            std::bind(&TopicMonitor::publish_diagnostics, this));

        // On-demand check: std_srvs/Trigger. Empty request; success = "all in sync".
        trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
            "check_time_sync",
            std::bind(&TopicMonitor::handle_trigger, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(),
                    "topic_monitor running: publishing /diagnostics every %.2fs; call the 'check_time_sync' "
                    "Trigger service for an on-demand pass/fail.",
                    publish_period);
    }

  private:
    // Helper function to create a QoS profile from configuration
    rclcpp::QoS create_qos_profile(const QoSConfig &config)
    {
        rclcpp::QoS qos_profile(config.history_depth);
        
        // Set reliability
        if (config.reliability == "best_effort")
        {
            qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        }
        else // Default to reliable
        {
            qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
        }
        
        // Set durability
        if (config.durability == "transient_local")
        {
            qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);
        }
        else // Default to volatile
        {
            qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        }
        
        return qos_profile;
    }

    void load_topics_from_file()
    {
        std::filesystem::path file_path;
        if (!relative_path_)
        {
            RCLCPP_INFO(this->get_logger(), "Using absolute path for topics file: %s", topics_file_.c_str());
            file_path = topics_file_;
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Using relative path for topics file: %s", topics_file_.c_str());
            std::string pkg_share_dir;
            try
            {
                pkg_share_dir = ament_index_cpp::get_package_share_directory("topic_monitor");
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error getting package share directory for 'topic_monitor': %s", e.what());
                rclcpp::shutdown();
                return;
            }
            catch (...)
            {
                RCLCPP_ERROR(this->get_logger(), "An unknown error occurred while getting package share directory for 'topic_monitor'.");
                rclcpp::shutdown();
                return;
            }
    
            file_path = pkg_share_dir;
            file_path /= topics_file_;
        }

        RCLCPP_INFO(this->get_logger(), "Attempting to load topics from: %s", file_path.string().c_str());

        // --- YAML PARSING LOGIC ---
        try
        {
            YAML::Node config = YAML::LoadFile(file_path.string());

            if (config["topics"])
            { // Expecting a top-level 'topics' key
                for (YAML::const_iterator it = config["topics"].begin(); it != config["topics"].end(); ++it)
                {
                    const YAML::Node &topic_node = *it;
                    if (topic_node["name"] && topic_node["type"])
                    {
                        std::string topic_name = topic_node["name"].as<std::string>();
                        std::string message_type = topic_node["type"].as<std::string>();
                        
                        // Parse QoS settings if present
                        QoSConfig qos_config;
                        if (topic_node["qos"])
                        {
                            const YAML::Node &qos_node = topic_node["qos"];
                            if (qos_node["history_depth"])
                            {
                                qos_config.history_depth = qos_node["history_depth"].as<size_t>();
                            }
                            if (qos_node["reliability"])
                            {
                                std::string reliability = qos_node["reliability"].as<std::string>();
                                if (reliability == "reliable" || reliability == "best_effort")
                                {
                                    qos_config.reliability = reliability;
                                }
                                else
                                {
                                    RCLCPP_WARN(this->get_logger(),
                                               "Invalid reliability value '%s' for topic '%s'. Using default 'reliable'. Valid options: 'reliable', 'best_effort'",
                                               reliability.c_str(), topic_name.c_str());
                                }
                            }
                            if (qos_node["durability"])
                            {
                                std::string durability = qos_node["durability"].as<std::string>();
                                if (durability == "volatile" || durability == "transient_local")
                                {
                                    qos_config.durability = durability;
                                }
                                else
                                {
                                    RCLCPP_WARN(this->get_logger(),
                                               "Invalid durability value '%s' for topic '%s'. Using default 'volatile'. Valid options: 'volatile', 'transient_local'",
                                               durability.c_str(), topic_name.c_str());
                                }
                            }
                        }

                        RCLCPP_INFO(this->get_logger(), 
                                    "Subscribing to topic: %s with type: %s (QoS: depth=%zu, reliability=%s, durability=%s)", 
                                    topic_name.c_str(), message_type.c_str(),
                                    qos_config.history_depth, qos_config.reliability.c_str(), qos_config.durability.c_str());
                        subscribe_to_topic(topic_name, message_type, qos_config);
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(),
                                    "Skipping malformed topic entry in YAML (missing 'name' or 'type'): %s",
                                    YAML::Dump(topic_node).c_str());
                    }
                }
            }
            else
            {
                RCLCPP_WARN(
                    this->get_logger(), "YAML file '%s' does not contain a top-level 'topics' array. No topics loaded.", file_path.string().c_str());
            }
        }
        catch (const YAML::BadFile &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open or parse YAML file: %s. Error: %s", file_path.string().c_str(), e.what());
            rclcpp::shutdown();
            return;
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error parsing YAML file '%s': %s", file_path.string().c_str(), e.what());
            rclcpp::shutdown();
            return;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "An unexpected error occurred during YAML processing: %s", e.what());
            rclcpp::shutdown();
            return;
        }
        // --- END YAML PARSING LOGIC ---
    }


    void subscribe_to_topic(const std::string &topic_name, const std::string &message_type, const QoSConfig &qos_config)
    {
        rclcpp::QoS qos_profile = create_qos_profile(qos_config);

        // Create a shared_ptr to TopicInfo on the heap
        std::shared_ptr<TopicInfo> current_info = std::make_shared<TopicInfo>(topic_name, message_type);
        current_info->qos_config = qos_config;

        // Add the shared_ptr to the vector
        topic_infos_.push_back(current_info);

        // The lambda captures current_info by value (copy of shared_ptr),
        // ensuring the TopicInfo object's lifetime is managed correctly.
        auto callback = [this, current_info](std::shared_ptr<rclcpp::SerializedMessage> msg)
        {
            rclcpp::Time now = this->now();
            rclcpp::Time stamp;
            bool have_stamp = extract_timestamp(msg, current_info->message_type, stamp);

            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!current_info->received_message_since_start)
            {
                current_info->first_received_walltime = now;
            }
            current_info->message_count++;
            current_info->received_message_since_start = true;
            current_info->last_received_walltime = now;

            if (have_stamp)
            {
                if (!current_info->last_timestamp.nanoseconds())  // first stamped message
                {
                    current_info->first_timestamp = stamp;
                }
                current_info->last_timestamp = stamp;
            }
        };

        // Store the subscription directly in the TopicInfo object via the shared_ptr
        current_info->subscription = this->create_generic_subscription(topic_name, message_type, qos_profile, callback);
    }

    bool extract_timestamp(std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string &message_type, rclcpp::Time &timestamp)
    {
        if (message_type == "sensor_msgs/msg/Imu")
        {
            sensor_msgs::msg::Imu imu_msg;
            rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;
            serializer.deserialize_message(msg.get(), &imu_msg);
            timestamp = imu_msg.header.stamp;
            return true;
        }
        else if (message_type == "nav_msgs/msg/Odometry")
        {
            nav_msgs::msg::Odometry odom_msg;
            rclcpp::Serialization<nav_msgs::msg::Odometry> serializer;
            serializer.deserialize_message(msg.get(), &odom_msg);
            timestamp = odom_msg.header.stamp;
            return true;
        }
        else if (message_type == "sensor_msgs/msg/Image")
        {
            sensor_msgs::msg::Image img_msg;
            rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
            serializer.deserialize_message(msg.get(), &img_msg);
            timestamp = img_msg.header.stamp;
            return true;
        }
        else if (message_type == "sensor_msgs/msg/NavSatFix")
        {
            sensor_msgs::msg::NavSatFix navsatfix_msg;
            rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serializer;
            serializer.deserialize_message(msg.get(), &navsatfix_msg);
            timestamp = navsatfix_msg.header.stamp;
            return true;
        }
        else if (message_type == "sensor_msgs/msg/PointCloud2")
        {
            sensor_msgs::msg::PointCloud2 point_msg;
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
            serializer.deserialize_message(msg.get(), &point_msg);
            timestamp = point_msg.header.stamp;
            return true;
        }
        else if (message_type == "sensor_msgs/msg/MagneticField")
        {
            sensor_msgs::msg::MagneticField point_msg;
            rclcpp::Serialization<sensor_msgs::msg::MagneticField> serializer;
            serializer.deserialize_message(msg.get(), &point_msg);
            timestamp = point_msg.header.stamp;
            return true;
        }
        else if (message_type == "sensor_msgs/msg/FluidPressure")
        {
            sensor_msgs::msg::FluidPressure out_msg;
            rclcpp::Serialization<sensor_msgs::msg::FluidPressure> serializer;
            serializer.deserialize_message(msg.get(), &out_msg);
            timestamp = out_msg.header.stamp;
            return true;
        }
        else if (message_type == "sensor_msgs/msg/BatteryState")
        {
            sensor_msgs::msg::BatteryState out_msg;
            rclcpp::Serialization<sensor_msgs::msg::BatteryState> serializer;
            serializer.deserialize_message(msg.get(), &out_msg);
            timestamp = out_msg.header.stamp;
            return true;
        }
        else if (message_type == "geometry_msgs/msg/TwistWithCovarianceStamped")
        {
            geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
            rclcpp::Serialization<geometry_msgs::msg::TwistWithCovarianceStamped> serializer;
            serializer.deserialize_message(msg.get(), &twist_msg);
            timestamp = twist_msg.header.stamp;
            return true;
        }
        // Add more messages as needed

        RCLCPP_WARN_ONCE(
            this->get_logger(), "Cannot extract timestamp for message type: %s. No specific deserializer implemented.", message_type.c_str());
        return false;
    }

    // Snapshot the live state into a DiagnosticArray. Also reports aggregate
    // counts and a one-line summary via the out-parameters. Caller must NOT hold
    // data_mutex_ (this method locks it).
    DiagnosticArray build_diagnostics(int &num_warn, int &num_error, std::string &summary, std::string &reference_topic)
    {
        DiagnosticArray array;
        array.header.stamp = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Time ref_time(0, 0, now.get_clock_type());
        reference_topic.clear();

        std::lock_guard<std::mutex> lock(data_mutex_);

        // Pick the reference topic: first in list that is fresh and stamped.
        std::shared_ptr<TopicInfo> reference_info;
        for (auto const &info : topic_infos_)
        {
            if (info->message_count == 0 || info->last_timestamp.nanoseconds() == 0)
            {
                continue;
            }
            if ((now - info->last_received_walltime).seconds() <= STALE_TIMEOUT_SECONDS)
            {
                reference_info = info;
                ref_time = info->last_timestamp;
                reference_topic = info->topic_name;
                break;
            }
        }

        num_warn = 0;
        num_error = 0;

        for (auto const &info : topic_infos_)
        {
            DiagnosticStatus status;
            status.name = diagnostic_name_prefix_ + ": " + info->topic_name;
            status.hardware_id = info->message_type;

            bool has_timestamp = (info->last_timestamp.nanoseconds() != 0);
            bool is_reference = (info == reference_info);

            double age = -1.0;
            double rate = 0.0;
            if (info->message_count > 0)
            {
                age = (now - info->last_received_walltime).seconds();
                double span = (info->last_received_walltime - info->first_received_walltime).seconds();
                rate = (span > 0.0 && info->message_count > 1) ? (info->message_count - 1) / span : 0.0;
            }
            double time_diff = 0.0;

            if (info->message_count == 0)
            {
                status.level = DiagnosticStatus::STALE;
                status.message = "No messages received since startup";
                num_error++;
            }
            else if (age > STALE_TIMEOUT_SECONDS)
            {
                status.level = DiagnosticStatus::STALE;
                status.message = "No message for " + fmt(age) + "s (stale threshold " + fmt(STALE_TIMEOUT_SECONDS) + "s)";
                num_error++;
            }
            else if (!has_timestamp)
            {
                status.level = DiagnosticStatus::ERROR;
                status.message = "Receiving, but no header timestamp could be extracted";
                num_error++;
            }
            else if (!reference_info)
            {
                status.level = DiagnosticStatus::OK;
                status.message = "Receiving (no reference topic available for comparison)";
            }
            else if (is_reference)
            {
                status.level = DiagnosticStatus::OK;
                status.message = "Reference topic";
            }
            else
            {
                time_diff = std::abs((info->last_timestamp - ref_time).seconds());
                if (time_diff > SYNC_THRESHOLD_ERROR_SECONDS)
                {
                    status.level = DiagnosticStatus::ERROR;
                    status.message = "Out of sync by " + fmt(time_diff) + "s (> error threshold " + fmt(SYNC_THRESHOLD_ERROR_SECONDS) + "s)";
                    num_error++;
                }
                else if (time_diff > SYNC_THRESHOLD_WARN_SECONDS)
                {
                    status.level = DiagnosticStatus::WARN;
                    status.message = "Offset " + fmt(time_diff) + "s (> warn threshold " + fmt(SYNC_THRESHOLD_WARN_SECONDS) + "s)";
                    num_warn++;
                }
                else
                {
                    status.level = DiagnosticStatus::OK;
                    status.message = "Synchronized (offset " + fmt(time_diff) + "s)";
                }
            }

            status.values.push_back(make_kv("message type", info->message_type));
            status.values.push_back(make_kv("message count", std::to_string(info->message_count)));
            status.values.push_back(make_kv("rate (Hz)", fmt(rate, 2)));
            status.values.push_back(make_kv("age (s)", age >= 0.0 ? fmt(age) : "n/a"));
            status.values.push_back(make_kv("has timestamp", has_timestamp ? "true" : "false"));
            status.values.push_back(make_kv("time diff (s)", has_timestamp ? fmt(time_diff) : "n/a"));
            status.values.push_back(make_kv("is reference", is_reference ? "true" : "false"));

            array.status.push_back(status);
        }

        summary = std::to_string(topic_infos_.size()) + " topics monitored, " + std::to_string(num_warn) +
                  " warnings, " + std::to_string(num_error) + " errors" +
                  (reference_topic.empty() ? "" : (" (ref: " + reference_topic + ")"));

        return array;
    }

    // Timer callback: publish the current diagnostics snapshot.
    void publish_diagnostics()
    {
        int num_warn = 0, num_error = 0;
        std::string summary, reference_topic;
        DiagnosticArray array = build_diagnostics(num_warn, num_error, summary, reference_topic);
        diagnostics_pub_->publish(array);
    }

    // Trigger service: report an on-demand pass/fail with a human-readable summary.
    void handle_trigger(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        int num_warn = 0, num_error = 0;
        std::string summary, reference_topic;
        DiagnosticArray array = build_diagnostics(num_warn, num_error, summary, reference_topic);

        // Publish immediately too, so any monitor reflects the freshly-checked state.
        diagnostics_pub_->publish(array);

        std::string detail = summary;
        for (auto const &s : array.status)
        {
            detail += "\n  [" + level_name(s.level) + "] " + s.name + " - " + s.message;
        }

        response->success = (num_warn == 0 && num_error == 0);
        response->message = detail;
        RCLCPP_INFO(this->get_logger(), "check_time_sync: %s", summary.c_str());
    }

    static KeyValue make_kv(const std::string &key, const std::string &value)
    {
        KeyValue kv;
        kv.key = key;
        kv.value = value;
        return kv;
    }

    static std::string level_name(unsigned char level)
    {
        switch (level)
        {
            case DiagnosticStatus::OK:    return "OK";
            case DiagnosticStatus::WARN:  return "WARN";
            case DiagnosticStatus::ERROR: return "ERROR";
            case DiagnosticStatus::STALE: return "STALE";
            default:                      return "UNKNOWN";
        }
    }

    static std::string fmt(double seconds, int precision = 3)
    {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.*f", precision, seconds);
        return std::string(buf);
    }

    // Parameters
    std::string topics_file_;
    bool relative_path_;
    double SYNC_THRESHOLD_WARN_SECONDS;
    double SYNC_THRESHOLD_ERROR_SECONDS;
    double STALE_TIMEOUT_SECONDS;
    std::string diagnostic_name_prefix_;

    std::mutex data_mutex_;
    std::vector<std::shared_ptr<TopicInfo>> topic_infos_;
    rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::TimerBase::SharedPtr diag_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TopicMonitor>();

    // Multi-threaded so the service / timer can run while topic callbacks keep
    // updating live state.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
