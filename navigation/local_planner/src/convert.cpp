#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/pose.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;

class Convert: public rclcpp::Node
{
	private:
		rclcpp::Publisher<robot_msgs::msg::Pose>::SharedPtr publisher;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subs;

		void odomCallback(const nav_msgs::msg::Odometry & msg);
	public:
		Convert(std::string name, std::string);

};

void Convert::odomCallback(const nav_msgs::msg::Odometry& msg)
{
	auto pose = robot_msgs::msg::Pose();
	pose.pose.x = msg.pose.pose.position.x;
	pose.pose.y = msg.pose.pose.position.y;
	
	tf2::Quaternion q(
			msg.pose.pose.orientation.x,
			msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z,
			msg.pose.pose.orientation.w
			);

	tf2::Matrix3x3 m(q);
	
	double roll, pitch, yaw;

	m.getRPY(roll, pitch, yaw);
	
	pose.pose.theta = yaw;

	publisher -> publish( pose );
	

}

Convert::Convert(std::string name="Convert", 
		 std::string odom_topic="/odom"): Node(name)
{
	RCLCPP_INFO( this -> get_logger(), "Demarrage du convertisseur position quarternion -> euler");

	subs = this -> create_subscription<nav_msgs::msg::Odometry>(odom_topic,
		       	10, 
			std::bind( &Convert::odomCallback, this, _1));

	publisher = this -> create_publisher<robot_msgs::msg::Pose>("pose2d", 10);
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin( std::make_shared<Convert> () );
	rclcpp::shutdown();

	return 0;
}
