#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/message_memory_strategy.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// ADD THESE MISSING MESSAGE TYPE HEADERS
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// ADD THIS MISSING SERIALIZATION HEADER
#include <rclcpp/serialization.hpp>
// Sometimes these provide dependencies for serialization to be fully visible
// #include <rclcpp/utilities.hpp> // Added in previous iteration, keep

#include <chrono>
#include <exception> // Explicitly include for std::exception
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include <vector>
#include <yaml-cpp/yaml.h>

// For dynamic message introspection
#include <rosidl_runtime_cpp/message_type_support_decl.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
// #include <rosidl_typesupport_introspection_cpp/traits.hpp> // Still commented out
#include <rosidl_typesupport_introspection_cpp/visibility_control.h>

using namespace std::chrono_literals;

// Structure to hold topic information
struct TopicInfo
{
    std::string topic_name;
    std::string message_type;
    int message_count = 0;
    rclcpp::Time first_timestamp;
    rclcpp::Time last_timestamp;
    bool received_message_since_start = false;
    std::shared_ptr<rclcpp::GenericSubscription> subscription;

    // Constructor to properly initialize all members and avoid warnings
    TopicInfo(std::string name, std::string type)
        : topic_name(std::move(name)), message_type(std::move(type)), message_count(0),
          last_timestamp(),                                          // Default constructs to an invalid/zero timestamp
          received_message_since_start(false), subscription(nullptr) // Initialize shared_ptr to nullptr
    {
    }
};


class TopicMonitor : public rclcpp::Node
{
  public:
    TopicMonitor() : Node("topic_monitor")
    {
        RCLCPP_INFO(this->get_logger(), "In topic_monitor constructor");

        this->declare_parameter<std::string>("topics_file", "topics.yaml");
        topics_file_ = this->get_parameter("topics_file").as_string();

        load_topics_from_file();

        // Timer to stop listening and report results after 5 seconds
        RCLCPP_INFO(this->get_logger(), "Starting 5 second timer to count messaged published from topics...");
        monitoring_start_time_ = this->now();
        timer_ = this->create_wall_timer(5s, std::bind(&TopicMonitor::report_and_shutdown, this));

        // Create an executor to process callbacks
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        // executor_->add_node(shared_from_this());

        // Spin in a separate thread to allow the main thread to continue and set up the timer
        spin_thread_ = std::thread([this]() { executor_->spin(); });
    }

    ~TopicMonitor()
    {
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
    }

    // New public method to start spinning the executor
    void start_monitoring() { executor_->add_node(shared_from_this()); }

  private:
    void load_topics_from_file()
    {
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

        std::filesystem::path file_path = pkg_share_dir;
        file_path /= topics_file_;

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

                        RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s with type: %s", topic_name.c_str(), message_type.c_str());
                        subscribe_to_topic(topic_name, message_type);
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


    void subscribe_to_topic(const std::string &topic_name, const std::string &message_type)
    {
        rclcpp::QoS qos_profile(10); // Standard QoS profile

        // Create a shared_ptr to TopicInfo on the heap
        std::shared_ptr<TopicInfo> current_info = std::make_shared<TopicInfo>(topic_name, message_type);

        // Add the shared_ptr to the vector
        topic_infos_.push_back(current_info);

        // The lambda captures current_info by value (copy of shared_ptr),
        // ensuring the TopicInfo object's lifetime is managed correctly.
        auto callback = [this, current_info](std::shared_ptr<rclcpp::SerializedMessage> msg)
        {
            current_info->message_count++;                     // Use -> operator for shared_ptr
            current_info->received_message_since_start = true; // Use -> operator

            rclcpp::Time stamp;
            if (extract_timestamp(msg, current_info->message_type, stamp)) // Use -> operator
            {
                if (!current_info->last_timestamp.nanoseconds()) // First message received for this topic
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
        std::string package_name = message_type.substr(0, message_type.find('/'));
        std::string msg_name = message_type.substr(message_type.find_last_of('/') + 1);

        // Workaround: We'll deserialize the message if it's one of the known types with a header.
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
        // Add more messages as needed

        RCLCPP_WARN_ONCE(
            this->get_logger(), "Cannot extract timestamp for message type: %s. No specific deserializer implemented.", message_type.c_str());
        return false;
    }


    void report_and_shutdown()
    {
        timer_->cancel(); // Stop the timer
        rclcpp::Time ref_time(0);

        RCLCPP_INFO(this->get_logger(), "--------------------------------------");
        RCLCPP_INFO(this->get_logger(), "---    Topic Monitoring Results    ---");
        RCLCPP_INFO(this->get_logger(), "--------------------------------------\n");

        // A threshold for "synchronization" difference. This can be tuned. // Parameterize this
        const double SYNC_THRESHOLD_WARN_SECONDS = 0.1;  // e.g., 100 milliseconds
        const double SYNC_THRESHOLD_ERROR_SECONDS = 1.0; // e.g., 100 milliseconds
        int num_warn = 0;
        int num_error = 0;
        for (auto const &info_ptr : topic_infos_) // Iterate over shared_ptrs
        {
            // Use -> to access members of TopicInfo via the shared_ptr
            if (info_ptr->message_count == 0)
            {
                RCLCPP_ERROR(this->get_logger(),
                             "Topic: %s (Type: %s) - ERROR: No messages published!\n",
                             info_ptr->topic_name.c_str(),
                             info_ptr->message_type.c_str());
                num_error++;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(),
                            "Topic: %s (Type: %s) - Messages received: %d",
                            info_ptr->topic_name.c_str(),
                            info_ptr->message_type.c_str(),
                            info_ptr->message_count);

                if (info_ptr->last_timestamp.nanoseconds() != 0)
                {
                    if (ref_time.nanoseconds() == 0) // First message with non zero timestamp
                    {
                        ref_time = info_ptr->last_timestamp;
                        RCLCPP_INFO(this->get_logger(), "Topic: %s - Set as reference timestamp\n", info_ptr->topic_name.c_str());
                    }
                    else
                    {
                        double time_diff = std::abs((info_ptr->last_timestamp - ref_time).seconds());
                        if (time_diff > SYNC_THRESHOLD_ERROR_SECONDS)
                        {
                            RCLCPP_ERROR(this->get_logger(),
                                         "Topic: %s - Error: Timestamps out of sync. Latest message time (%.9f) is %.3f seconds from reference "
                                         "topic time (%.9f). Expected within %.3f s.",
                                         info_ptr->topic_name.c_str(),
                                         info_ptr->last_timestamp.seconds(),
                                         time_diff,
                                         ref_time.seconds(),
                                         SYNC_THRESHOLD_ERROR_SECONDS);
                            num_error++;
                        }
                        else if (time_diff > SYNC_THRESHOLD_WARN_SECONDS)
                        {
                            RCLCPP_WARN(this->get_logger(),
                                        "Topic: %s - WARNING: Timestamps potentially out of sync. Latest message time (%.9f) is \033[31m%.3f\033[33m "
                                        "seconds from reference "
                                        "topic time (%.9f). Expected within %.3f s. \033[31m(Possibly unsynchronized or very low frequency)\033[0m\n",
                                        info_ptr->topic_name.c_str(),
                                        info_ptr->last_timestamp.seconds(),
                                        time_diff,
                                        ref_time.seconds(),
                                        SYNC_THRESHOLD_WARN_SECONDS);
                            num_warn++;
                        }
                        else
                        {
                            RCLCPP_INFO(this->get_logger(),
                                        "Topic: %s - Timestamps appear synchronized with end time difference \033[32m%.3f\033[0m s\n",
                                        info_ptr->topic_name.c_str(),
                                        time_diff);
                        }
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(),
                                "Topic: %s - Could not check timestamp synchronization. No header found or first message not received properly.\n",
                                info_ptr->topic_name.c_str());
                    num_error++;
                }
            }
        }
        // \033[0m -> Reset
        // \033[31m -> Red
        // \033[32m -> Green
        // \033[33m -> Yellow
        RCLCPP_INFO(this->get_logger(),
                    "Warnings: %s%d%s, Errors: %s%d%s",
                    (num_warn > 0) ? "\033[33m" : "\033[32m",
                    num_warn,
                    "\033[0m", // if >0 then yellow else green
                    (num_error > 0) ? "\033[31m" : "\033[32m",
                    num_error,
                    "\033[0m"); // if >0 then red else green
        RCLCPP_INFO(this->get_logger(), "-------------------------------------");
        RCLCPP_INFO(this->get_logger(), "---      Monitoring Complete      ---");
        RCLCPP_INFO(this->get_logger(), "-------------------------------------");
        rclcpp::shutdown();
    }

    std::string topics_file_;
    std::vector<std::shared_ptr<TopicInfo>> topic_infos_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time monitoring_start_time_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread spin_thread_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TopicMonitor>();
    // The node will manage its own spinning via the MultiThreadedExecutor in a separate thread.
    // main will just wait for ROS to shutdown, which is triggered by report_and_shutdown().
    node->start_monitoring();

    while (rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep briefly to avoid busy-waiting
    }

    rclcpp::shutdown();
    return 0;
}