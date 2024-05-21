#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <math.h>
#include <yolov8_msgs/srv/target_pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define R_VEL   0.2         // rotate velocity
#define F_VEL   0.2         // forward velocity
// #define HEIGHT  480
// #define WIDTH   640

struct HumanPose {
    bool valid;
    std::tuple<double, double, double> goal; // (x, y, z) 좌표
};

geometry_msgs::msg::Twist velOutput;

HumanPose p;
// const double target_distance = 1.0; // 목표 거리 (예: 1미터)

class HumanFollower : public rclcpp::Node {
public:
    HumanFollower() : Node("human_follower") {
        // // QoS Profile 설정
        // rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
        // custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        // custom_qos_profile.depth = 10;
        // custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

        RCLCPP_INFO(this->get_logger(), "Initialized node");
        srv_client = this->create_client<yolov8_msgs::srv::TargetPose>("yolo/target_pose"); 
        // , std::bind(&HumanFollower::handle_request, this, std::placeholders::_1, std::placeholders::_2), custom_qos_profile);
        pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&HumanFollower::check_and_request, this));
    }

private:
    void check_and_request() {
        if (!p.valid) {
            request_target();
        }
    }

    void request_target() {
        auto request = std::make_shared<yolov8_msgs::srv::TargetPose::Request>();
        // std::shared_ptr<yolov8_msgs::srv::TargetPose::Response> response;
        while (!srv_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }

        auto result = srv_client->async_send_request(request);
        result.wait();
        try {
            auto response = result.get();
            handle_response(response);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

    void handle_response(
        const std::shared_ptr<yolov8_msgs::srv::TargetPose::Response> response){
        try {
            // Adjust the x-coordinate to be centered
            p.goal = std::make_tuple(response->x, response->y, response->z);
            p.valid = true;

            RCLCPP_INFO(this->get_logger(), "Target Pose: x=%f, y=%f, z=%f", std::get<0>(p.goal), std::get<1>(p.goal), std::get<2>(p.goal));

            // Update the robot's velocity command
            if (p.valid) {
                GettingHuman();
            } else {
                LostHuman();
            }
        } catch (const std::exception &e) {
            p.valid = false;
            RCLCPP_ERROR(this->get_logger(), "Failed to get target pose: %s", e.what());
            LostHuman();
        }
    }

    void GettingHuman() {
        RCLCPP_INFO(this->get_logger(), "I DETECT HUMAN");
        double person_x = std::get<0>(p.goal);
        double person_y = std::get<1>(p.goal);

        // Robot's rotation control: use y value to align to the center of the camera
        if (person_y < -0.05)
            velOutput.angular.z = R_VEL;
        else if (person_y > 0.05)
            velOutput.angular.z = -R_VEL;
        else
            velOutput.angular.z = 0;

        // Robot's forward/backward control: use x value to maintain a certain distance
        if (person_x > 0.5) {
            RCLCPP_INFO(this->get_logger(), "FORWARD");
            velOutput.linear.x = F_VEL;
         }// else if (person_x < - 0.05) {
        //     RCLCPP_INFO(this->get_logger(), "BACKWARD");
        //     velOutput.linear.x = - F_VEL;
        else {
            RCLCPP_INFO(this->get_logger(), "STOP");
            velOutput.linear.x = 0;
        }

        pub->publish(velOutput);
    }

    void LostHuman() {
        RCLCPP_ERROR(this->get_logger(), "I LOST HUMAN");
        velOutput.linear.x = 0;
        velOutput.angular.z = 0;
        pub->publish(velOutput);
    }

    rclcpp::Client<yolov8_msgs::srv::TargetPose>::SharedPtr srv_client;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto turtlebot3_controller = std::make_shared<HumanFollower>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(turtlebot3_controller);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}