#include <memory>
#include <functional>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/pose.hpp>

using std::placeholders::_1;
class Move: public rclcpp::Node
{
	private:
		rclcpp::Subscription<robot_msgs::msg::Pose>::SharedPtr robot_pose;

		rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal;

		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd;

		geometry_msgs::msg::Point current_goal;

		void robotPoseCallback(const robot_msgs::msg::Pose& pose);

		void goalCallback(const geometry_msgs::msg::Point& goal);

		float kp = 0.5;
	public:
		Move(std::string name);
};

Move::Move(std::string name="Move"): Node(name)
{
	RCLCPP_INFO( this -> get_logger(), "Demarrage du topic move");

	current_goal.x = 0.2702;
	current_goal.y = -0.2401;

	robot_pose = this -> create_subscription<robot_msgs::msg::Pose>(
			"pose2d",
			10, 
			std::bind(&Move::robotPoseCallback, this, _1));

	goal = this -> create_subscription<geometry_msgs::msg::Point>(
			"goal",
			10,
			std::bind(&Move::goalCallback, this, _1));

	cmd =  this -> create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

}

void Move::robotPoseCallback(const robot_msgs::msg::Pose& pose)
{
	float new_x = current_goal.x - pose.pose.x;
	float new_y = current_goal.y - pose.pose.y;
	

	float angle_to_goal = atan2(new_y, new_x);

	auto cmd_vel = geometry_msgs::msg::Twist();
	
	float diff_goal_x = std::abs(pose.pose.x - current_goal.x);
	float diff_goal_y = std::abs(pose.pose.y - current_goal.y);
	float diff_angle = angle_to_goal - pose.pose.theta ;
	
	RCLCPP_INFO( this -> get_logger(), "%f, %f, %f", diff_goal_x, diff_goal_y, diff_angle);

	if ( diff_goal_x < 0.1 && diff_goal_y )
	{
		cmd_vel.linear.x = 0.0;
		cmd_vel.angular.z = 0.0;
	}

	else if ( std::abs( diff_angle ) > 0.1 )
	{
		cmd_vel.linear.x = 0.0;
		cmd_vel.angular.z = kp * diff_angle;
	}
	else
	{
		cmd_vel.linear.x = 0.2;
		cmd_vel.angular.z = 0.0;
	}

	cmd -> publish(cmd_vel);

}

void Move::goalCallback(const geometry_msgs::msg::Point& goal)
{
	RCLCPP_INFO( this -> get_logger(), "Current goal is : %f, %f, %f", goal.x, goal.y, goal.z);

	current_goal = goal;
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin( std::make_shared<Move> () );
	rclcpp::shutdown();

	return 0;
}

		
