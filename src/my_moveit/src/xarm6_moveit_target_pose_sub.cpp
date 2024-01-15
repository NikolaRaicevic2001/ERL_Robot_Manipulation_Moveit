#include <memory>
#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/bool.hpp>
#include <xarm_msgs/srv/plan_pose.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>

using std::placeholders::_1;

#define SERVICE_CALL_FAILED 999

std::shared_ptr<rclcpp::Node> node;

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[test_xarm_planner_node_pose_sub] Ctrl-C caught, exit process...\n");
    exit(-1);
}

template <typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr>
int call_request(std::shared_ptr<ServiceT> client, SharedRequest req)
{
    bool is_try_again = false;
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(1);
        }
        if (!is_try_again)
        {
            is_try_again = true;
            RCLCPP_WARN(node->get_logger(), "service %s not available, waiting ...", client->get_service_name());
        }
    }
    auto result_future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service %s", client->get_service_name());
        return SERVICE_CALL_FAILED;
    }
    auto res = result_future.get();
    RCLCPP_INFO(node->get_logger(), "call service %s, success=%d", client->get_service_name(), res->success);
    return res->success;
}

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("minimal_subscriber"),
          pose_plan_client_(this->create_client<xarm_msgs::srv::PlanPose>("xarm_pose_plan")),
          exec_plan_client_(this->create_client<xarm_msgs::srv::PlanExec>("xarm_exec_plan"))
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
    {
        std::shared_ptr<xarm_msgs::srv::PlanPose::Request> pose_plan_req = std::make_shared<xarm_msgs::srv::PlanPose::Request>();
        std::shared_ptr<xarm_msgs::srv::PlanExec::Request> exec_plan_req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();

        exec_plan_req->wait = true;

        pose_plan_req->target = msg->pose;
        call_request(pose_plan_client_, pose_plan_req);
        call_request(exec_plan_client_, exec_plan_req);
    }

    mutable rclcpp::Client<xarm_msgs::srv::PlanPose>::SharedPtr pose_plan_client_;
    mutable rclcpp::Client<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node = rclcpp::Node::make_shared("test_xarm_planner_node_pose_sub", node_options);
    RCLCPP_INFO(node->get_logger(), "test_xarm_planner_node_pose_sub start");
    signal(SIGINT, exit_sig_handler);

    // Create a subscriber to the target_pose topic
        
    while (rclcpp::ok())
    {
        rclcpp::spin(std::make_shared<MinimalSubscriber>());
    }
    
    // rclcpp::shutdown();

    RCLCPP_INFO(node->get_logger(), "test_xarm_planner_node_pose_sub over");
    return 0;
}


