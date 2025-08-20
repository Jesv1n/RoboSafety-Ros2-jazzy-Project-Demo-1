/**
 * @file cpp_minimal_publisher.cpp
 * @brief Demonstrates publishing string messages to a ROS 2 topic.
 *
 * Description: Demonstrates the basics of publishing messages within
 * the ROS 2 framework.
 *
 * The core functionality of this publisher is to repeatedly send out
 * string messages at a fixed frequency.
 *
 * -------
 * Subscription Topics:
 *   None
 * -------
 * Publishing Topics:
 *   String message
 *   /cpp_example_topic - std_msgs/String
 * -------
 * @author Addison Sears-Collins
 * @date November 5, 2024
 */
 
#include <rclcpp/rclcpp.hpp>// ROS 2 C++ client library
#include <std_msgs/msg/string.hpp>// Standard message type for strings
 
using namespace std::chrono_literals; // Handles time duration
 
/**
 * @class MinimalCppPublisher
 * @brief Defines a minimal ROS 2 publisher node.
 *
 * This class inherits from rclcpp::Node and demonstrates creating a
 * publisher, publishing messages, and using a timer callback in ROS 2.
 */
class MinimalCppPublisher : public rclcpp::Node
{
public:
    /**
     * @brief Constructs a MinimalCppPublisher node.
     *
     * Sets up a publisher for 'std_msgs::msg::String' messages on
     * the "/cpp_example_topic" topic and initializes a timer to call
     * the timerCallback method.
     */
    MinimalCppPublisher() : Node("minimal_cpp_publisher"), count_(0)
    {
        // Create a publisher object for sending string messages
        // with a queue size of 10.
        publisher_ = create_publisher<std_msgs::msg::String>(
          "/cpp_example_topic", 10);
 
        // Set up a timer to call the timerCallback function
        timer_ = create_wall_timer(500ms,
          std::bind(&MinimalCppPublisher::timerCallback, this));
 
        RCLCPP_INFO(get_logger(), "Publishing at 2 Hz");
    }
 
    /**
     * @brief Timer callback function.
     *
     * This method is called at a fixed interval.
     * It publishes a string message
     * containing "Hello World" followed by a sequence number.
     *
     *  @return void
     */
    void timerCallback()
    {
        // Create a new String message object.
        auto message = std_msgs::msg::String();
        message.data = "Hello World! " +  std::to_string(count_++);
 
        // Publish the message
        publisher_->publish(message);
    }
 
private:
  // Member variables
  size_t count_;  // Keep track of the number of messages published
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    publisher_;  // The publisher object
  rclcpp::TimerBase::SharedPtr timer_;  // Timer
};
 
/**
 * @brief Main function.
 *
 * Initializes the ROS 2 system and runs the minimal_cpp_publisher node.
 * It keeps the node alive until it is manually terminated.
 */
int main(int argc, char * argv[])
{
 
  // Initialize ROS 2.
  rclcpp::init(argc, argv);
 
  // Create an instance of the MinimalCppPublisher node and keep it running.
  auto minimal_cpp_publisher_node = std::make_shared<MinimalCppPublisher>();
  rclcpp::spin(minimal_cpp_publisher_node);
 
  // Shutdown ROS 2 upon node termination.
  rclcpp::shutdown();
 
  // End of program.
  return 0;
}