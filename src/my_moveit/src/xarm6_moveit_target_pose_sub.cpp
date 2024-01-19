#include <signal.h>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <xarm_msgs/srv/plan_pose.hpp>
#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <xarm_msgs/srv/plan_single_straight.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#define SERVICE_CALL_FAILED 999

std::shared_ptr<rclcpp::Node> node;

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[test_xarm_planner_node_pose] Ctrl-C caught, exit process...\n");
    exit(-1);
}

// Declare a global variable to store received pose values 
geometry_msgs::msg::PoseStamped::SharedPtr g_received_pose;

template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr>
int call_request(std::shared_ptr<ServiceT> client, SharedRequest req)
{
    bool is_try_again = false;
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(1);
        }
        if (!is_try_again) {
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
        : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("target_pose", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        g_received_pose = msg;
        return;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{	
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);    
    node = rclcpp::Node::make_shared("test_xarm_planner_node_pose", node_options);
    RCLCPP_INFO(node->get_logger(), "test_xarm_planner_node_pose start");
    signal(SIGINT, exit_sig_handler);

    int dof;
    node->get_parameter_or("dof", dof, 6);
    RCLCPP_INFO(node->get_logger(), "namespace: %s, dof: %d", node->get_namespace(), dof);

    // Create a subscriber to the target_pose topic
    auto minimal_subscriber = std::make_shared<MinimalSubscriber>();

    rclcpp::Client<xarm_msgs::srv::PlanPose>::SharedPtr pose_plan_client_ = node->create_client<xarm_msgs::srv::PlanPose>("xarm_pose_plan");
    rclcpp::Client<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_client_ = node->create_client<xarm_msgs::srv::PlanExec>("xarm_exec_plan");

    std::shared_ptr<xarm_msgs::srv::PlanPose::Request> pose_plan_req = std::make_shared<xarm_msgs::srv::PlanPose::Request>();;
    std::shared_ptr<xarm_msgs::srv::PlanExec::Request> exec_plan_req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();;

    exec_plan_req->wait = true;

    geometry_msgs::msg::Pose target_pose1;
    target_pose1.position.x = 0.3;
	target_pose1.position.y = -0.1;
	target_pose1.position.z = 0.2;
	target_pose1.orientation.x = 1;
	target_pose1.orientation.y = 0;
	target_pose1.orientation.z = 0;
	target_pose1.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose2;
    target_pose2.position.x = 0.3;
	target_pose2.position.y = 0.1;
	target_pose2.position.z = 0.2;
	target_pose2.orientation.x = 1;
	target_pose2.orientation.y = 0;
	target_pose2.orientation.z = 0;
	target_pose2.orientation.w = 0;
    
    while (rclcpp::ok())
    {
        pose_plan_req->target = target_pose1;
        call_request(pose_plan_client_, pose_plan_req);
        call_request(exec_plan_client_, exec_plan_req);

        pose_plan_req->target = target_pose2;
        call_request(pose_plan_client_, pose_plan_req);
        call_request(exec_plan_client_, exec_plan_req);

        // Explicitly call spin_once to handle any pending callbacks
        rclcpp::spin_some(minimal_subscriber);

        if(g_received_pose != nullptr){
            RCLCPP_INFO(node->get_logger(), "Message received: X = %f | Y = %f | Z = %f",g_received_pose->pose.position.x, g_received_pose->pose.position.y, g_received_pose->pose.position.z);
            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = g_received_pose->pose.position.x;
            target_pose.position.y = g_received_pose->pose.position.y;
            target_pose.position.z = g_received_pose->pose.position.z;
            target_pose.orientation.x = g_received_pose->pose.orientation.x;
            target_pose.orientation.y = g_received_pose->pose.orientation.y;
            target_pose.orientation.z = g_received_pose->pose.orientation.z;
            target_pose.orientation.w = g_received_pose->pose.orientation.w;

            pose_plan_req->target = target_pose;
            call_request(pose_plan_client_, pose_plan_req);
            call_request(exec_plan_client_, exec_plan_req);

            // Reset the global variable after processing
            g_received_pose = nullptr; // Assuming g_received_pose is a std::shared_ptr<geometry_msgs::msg::PoseStamped>

        }

        // Add a sleep to prevent high CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(node->get_logger(), "test_xarm_planner_node_pose over");
    return 0;
}