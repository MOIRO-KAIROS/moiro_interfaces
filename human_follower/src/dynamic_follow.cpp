#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <math.h>
#include <moiro_interfaces/srv/target_pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define R_VEL   0.3         // rotate velocity
#define F_VEL   0.1         // forward velocity
// #define HEIGHT  480
// #define WIDTH   640

struct HumanPose {
    bool valid = false;
    std::tuple<double, double, double> goal; // (x, y, z) 좌표
};

geometry_msgs::msg::Twist velOutput;

HumanPose p;
// const double target_distance = 1.0; // 목표 거리 (예: 1미터)

class HumanFollower : public rclcpp::Node {
public:
    HumanFollower() : Node("human_follower") {
        RCLCPP_INFO(this->get_logger(), "Initialized node");

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        srv_client = this->create_client<moiro_interfaces::srv::TargetPose>("vision/target_pose");

        // 주기적으로 check_and_request 호출
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&HumanFollower::check_and_request, this)
        );
    }

private:
    void check_and_request() {
        if (!p.valid) {
            request_target();
        }
    }

    void request_target() {
        auto request = std::make_shared<moiro_interfaces::srv::TargetPose::Request>();
        request->prepared = true;
        auto future = srv_client->async_send_request(
            request, std::bind(&HumanFollower::handle_response, this, std::placeholders::_1)
        );
    }

    void handle_response(rclcpp::Client<moiro_interfaces::srv::TargetPose>::SharedFuture future) {
        try {
            auto response = future.get();
            p.goal = std::make_tuple(response->x, response->y, response->z);
            p.valid = true;
            RCLCPP_INFO(this->get_logger(), "Lost Human: %d", (response->status));

            RCLCPP_INFO(this->get_logger(), "Target Pose: x=%f, y=%f, z=%f", std::get<0>(p.goal), std::get<1>(p.goal), std::get<2>(p.goal));

            if (p.valid && response->status) {
                GettingHuman();
            } else {
                LostHuman();
            }
            p.valid = false;
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

        // 로봇의 회전 제어: y 값을 사용하여 카메라 중앙에 정렬
        if (person_y < -0.05)
            velOutput.angular.z = - R_VEL;
        else if (person_y > 0.05)
            velOutput.angular.z = R_VEL;
        else
            velOutput.angular.z = 0;
        
        // 로봇의 전진/후진 제어: x 값을 사용하여 특정 거리 유지
        if (person_x > 0.5) {
            RCLCPP_INFO(this->get_logger(), "FORWARD");
            velOutput.linear.x = F_VEL;
        } else {
            RCLCPP_INFO(this->get_logger(), "STOP");
            velOutput.linear.x = 0;
        }
        
        RCLCPP_INFO(this->get_logger(), "linear - x : %f  angular - z: %f ", velOutput.linear.x, velOutput.angular.z );
        pub_->publish(velOutput);
    }

    void LostHuman() {
        RCLCPP_ERROR(this->get_logger(), "I LOST HUMAN");
        velOutput.linear.x = 0;
        velOutput.angular.z = 0;
        pub_->publish(velOutput);
    }

    rclcpp::Client<moiro_interfaces::srv::TargetPose>::SharedPtr srv_client;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
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