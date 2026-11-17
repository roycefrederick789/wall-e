#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/hand_detection.hpp"  // CHANGE
#include "tutorial_interfaces/msg/face_detection.hpp"  // CHANGE
#include "tutorial_interfaces/msg/servos_command2.hpp" // CHANGE

using std::placeholders::_1;
using namespace std::chrono_literals;

class VisionSubscriber : public rclcpp::Node
{
public:
    VisionSubscriber()
        : Node("vision_subscriber"),
          last_ros_command_time_(std::chrono::steady_clock::now() - std::chrono::milliseconds(200))
    {
        hand_subscription_ = this->create_subscription<tutorial_interfaces::msg::HandDetection>(
            "hand_coord_host2laptop", 10, std::bind(&VisionSubscriber::hand_callback, this, _1));

        face_subscription_ = this->create_subscription<tutorial_interfaces::msg::FaceDetection>(
            "face_coord_host2laptop", 10, std::bind(&VisionSubscriber::face_callback, this, _1));

        servos_publisher_ = this->create_publisher<tutorial_interfaces::msg::ServosCommand2>("servos_laptop2esp32", 10);

        check_servos_subscription_ = this->create_subscription<tutorial_interfaces::msg::ServosCommand2>("resend_esp2host", 10, std::bind(&VisionSubscriber::esp_topic_callback, this, _1));
    }

private:
    bool follow_head_ = false;
    bool hand_waving_ = false;
    bool face_detected_ = false;
    double lastYaw = 0;
    double lastPitch = 0;
    std::chrono::steady_clock::time_point lose_face_start_time_;
    std::chrono::steady_clock::time_point last_scan_time_;

    // Minimum interval between servo commands
    const std::chrono::milliseconds ros_command_interval_{200};

    // Time when the last command was published
    std::chrono::steady_clock::time_point last_ros_command_time_;

    bool lose_face_ = false;

    int scan_index_ = 0;
    const double scan_yaw_[7] = {-20, -20, -20, 0, 20, 20, 20};
    const double scan_pitch_[7] = {20, 0, -20, -20, -20, 0, 20};
    void publish_servo_target(double yaw, double pitch)
    {
        auto current_ros_time = std::chrono::steady_clock::now();
        // Check whether enough time has passed
        // if (current_ros_time - last_ros_command_time_ < ros_command_interval_)
        // {
        //     return;
        // }

        auto message = tutorial_interfaces::msg::ServosCommand2();

        message.new_yaw = static_cast<float>(yaw);
        message.new_pitch = static_cast<float>(pitch);
        message.new_roll = 0.0f;

        servos_publisher_->publish(message);
        // Update the last publishing time
        last_ros_command_time_ = current_ros_time;

        RCLCPP_INFO(
            this->get_logger(),
            "Sending: yaw=%.2f, pitch=%.2f",
            yaw, pitch);
    }

    void scan_for_face()
    {
        auto current_time = std::chrono::steady_clock::now();

        // Start the search timer when the face is first lost
        if (!lose_face_)
        {
            lose_face_ = true;
            lose_face_start_time_ = current_time;
            last_scan_time_ = current_time;

            scan_index_ = 0;

            RCLCPP_INFO(
                this->get_logger(),
                "Face lost. Starting scan.");
        }

        // Stop searching after 20 seconds
        std::chrono::duration<double> elapsed =
            current_time - lose_face_start_time_;

        if (elapsed.count() > 10.0)
        {
            follow_head_ = false;
            lose_face_ = false;

            RCLCPP_INFO(
                this->get_logger(),
                "Face search timeout.");

            publish_servo_target(0, 0);

            return;
        }

        // Move to the next scanning position every 2 seconds
        std::chrono::duration<double> scan_elapsed =
            current_time - last_scan_time_;

        if (scan_elapsed.count() >= 1.0)
        {
            lastYaw = scan_yaw_[scan_index_];
            lastPitch = scan_pitch_[scan_index_];

            RCLCPP_INFO(this->get_logger(), "Scanning head");
            publish_servo_target(lastYaw, lastPitch);

            scan_index_ = (scan_index_ + 1) % 7;

            last_scan_time_ = current_time;
        }
    }

    // bool latest_msg_ = false;
    void hand_callback(const tutorial_interfaces::msg::HandDetection &msg)
    {
        /*RCLCPP_INFO(
            this->get_logger(),
            "Hand: x=%.3f, y=%.3f, waving=%s",
            static_cast<double>(msg.x),
            static_cast<double>(msg.y),
            msg.waving ? "true" : "false");*/

        double hand_x = msg.x;
        double hand_y = msg.y;
        hand_waving_ = msg.waving;
        if (hand_waving_)
        {
            follow_head_ = true;
        }
    }

    void face_callback(const tutorial_interfaces::msg::FaceDetection &msg)
    {
        /*RCLCPP_INFO(
            this->get_logger(),
            "Face: x=%.3f, y=%.3f, detected=%s",
            static_cast<double>(msg.x),
            static_cast<double>(msg.y),
            msg.detected ? "true" : "false");*/

        double face_x = msg.x;
        double face_y = msg.y;
        face_detected_ = msg.detected;

        if (face_detected_ == true && follow_head_ == true)
        {
            double e_x = 0.5 - face_x;
            double e_y = 0.5 - face_y;
            bool adjust_servos = false;
            double del_yaw = 0;
            double del_pitch = 0;
            lose_face_ = false;

            const double Kp = 8.0;
            const double deadband = 0.1;

            // if the values exceed the deadband
            if (std::abs(e_x) > deadband)
            {
                del_yaw = Kp * e_x;
            }
            if (std::abs(e_y) > deadband)
            {
                del_pitch = Kp * e_y;
            }
            if (del_yaw != 0.0 || del_pitch != 0.0)
            {
                // adjust_servos = true;
                lastYaw = lastYaw + del_yaw;
                lastPitch = lastPitch + del_pitch;

                publish_servo_target(lastYaw, lastPitch);
                RCLCPP_INFO(this->get_logger(), "Following head");
                // auto message = tutorial_interfaces::msg::ServosCommand2();
                // message.new_yaw = static_cast<float>(lastYaw);
                // message.new_pitch = static_cast<float>(lastPitch);
                // message.new_roll = 0.0f;

                // // RCLCPP_INFO(this->get_logger(), "Publishing: %d", message.data);
                // RCLCPP_INFO(this->get_logger(), "Publishing: [%.3f, %.3f, %.3f]",
                //             del_yaw, del_pitch, 0.0);

                // servos_publisher_->publish(message);
            }
        }
        else if (follow_head_ == true && face_detected_ == false)
        {
            // Face lost: start or continue scanning
            scan_for_face();
        }
    }

    void esp_topic_callback(const tutorial_interfaces::msg::ServosCommand2 &msg) const // CHANGE
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "I heard: d_yaw=" << msg.new_yaw << ", d_pitch=" << msg.new_pitch << ", d_roll=" << msg.new_roll); // CHANGE
    }

    rclcpp::Subscription<tutorial_interfaces::msg::HandDetection>::SharedPtr hand_subscription_;
    rclcpp::Subscription<tutorial_interfaces::msg::FaceDetection>::SharedPtr face_subscription_;
    rclcpp::Publisher<tutorial_interfaces::msg::ServosCommand2>::SharedPtr servos_publisher_;
    rclcpp::Subscription<tutorial_interfaces::msg::ServosCommand2>::SharedPtr check_servos_subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionSubscriber>());
    rclcpp::shutdown();
    return 0;
}
