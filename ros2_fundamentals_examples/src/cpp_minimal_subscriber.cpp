/**
 * Minimal C++ Subscriber Node
 * @file cpp_minimal_subscriber.cpp
 * @brief Demonstrates subscribing to string messages from a ROS 2 topic.
 * @author Jesvin Paul
 * @date 2025-08-20
 * 
 * ---------------------
 * Subscription Topics:
 * String Message
 *  /cpp_example_topic -std_msgs/String
 * ----------------------
 * Publishing Topics:
 * NONE
 */


 #include <rclcpp/rclcpp.hpp> // ROS 2 C++ client library
 #include <std_msgs/msg/string.hpp> // ROS 2 String message type

using std::placeholders::_1; // Placeholder for message callback

class MInimalCppSubscriber : public rclcpp::Node
{
    public:
      MInimalCppSubscriber():Node("minimal_cpp_subscriber")
      {
        subscriber = create_subscription<std_msgs::msg::String>(
          "/cpp_example_topic",
          10,
          std::bind(
            &MInimalCppSubscriber::topicCallback, 
            this, 
            _1
        )
      );
      
    }


    void topicCallback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO_STREAM(get_logger(), "I Heard: " << msg->data.c_str());

    }
private:
//member variable
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber; // Subscription pointer
    // The SharedPtr is used to manage the lifetime of the subscription object.
    
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv); // Initialize the ROS 2 client library
    auto minimal_cpp_subscriber_node = std::make_shared<MInimalCppSubscriber>(); // Create an instance of the subscriber node
    rclcpp::spin(minimal_cpp_subscriber_node); // Spin the node to process callbacks
    rclcpp::shutdown(); // Shutdown the ROS 2 client library
    return 0; // Exit the program
}

