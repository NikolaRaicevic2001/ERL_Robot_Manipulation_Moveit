import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from xarm_msgs.srv import PlanPose, PlanExec
from geometry_msgs.msg import Pose

SERVICE_CALL_FAILED = 999

class TestXarmPlannerNodePose(Node):

    def __init__(self):
        super().__init__('test_xarm_planner_node_pose')
        self.get_logger().info("test_xarm_planner_node_pose start")

        signal.signal(signal.SIGINT, self.exit_sig_handler)

        self.declare_parameter('dof', 6)
        dof = self.get_parameter('dof').value   
        
        self.pose_plan_client = self.create_client(PlanPose, 'xarm_pose_plan')
        self.exec_plan_client = self.create_client(PlanExec, 'xarm_exec_plan')

        self.pose_plan_req = PlanPose.Request()
        self.exec_plan_req = PlanExec.Request()

        self.exec_plan_req.wait = True

        self.target_poses = [
            self.create_target_pose(0.3, -0.1, 0.2),
            self.create_target_pose(0.3, 0.1, 0.2),
            self.create_target_pose(0.3, 0.1, 0.4),
            self.create_target_pose(0.3, -0.1, 0.4)
        ]

        self.timer = self.create_timer(1, self.timer_callback)

    def exit_sig_handler(self, signum, frame):
        self.get_logger().info("Ctrl-C caught, exit process...")
        self.destroy_node()
        rclpy.shutdown()

    def create_target_pose(self, x, y, z):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        return pose

    def call_request(self, client, req):
        is_try_again = False
        # node.get_logger().info("loc 1")
        while not client.wait_for_service(1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for the service. Exiting.")
                return SERVICE_CALL_FAILED
            if not is_try_again:
                is_try_again = True
                self.get_logger().warn("Service %s not available, waiting ...", client.srv_name)

        future = client.call_async(req)
        if rclpy.spin_until_future_complete(self, future) != rclpy.future.FutureReturnCode.SUCCESS:
            self.get_logger().error("Failed to call service %s", client.srv_name)
            return SERVICE_CALL_FAILED
        res = future.result()
        self.get_logger().info("Call service %s, success=%d", client.srv_name, res.success)
        return res.success

    def timer_callback(self):
        for target_pose in self.target_poses:
            self.pose_plan_req.target = target_pose
            self.call_request(self.pose_plan_client, self.pose_plan_req)
            self.call_request(self.exec_plan_client, self.exec_plan_req)


def main(args=None):
    rclpy.init(args=args)
    node = TestXarmPlannerNodePose()
    node.get_logger().info("in main")
    rclpy.spin(node)
    node.get_logger().info("test_xarm_planner_node_pose over")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
