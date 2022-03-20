#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ImuAdapter : public rclcpp::Node
{
  public:
    ImuAdapter()
    : Node("imu_adapter")
    {
      subscription = this->create_subscription<sensor_msgs::msg::Imu>("imu_plugin/out", rclcpp::SensorDataQoS(rclcpp::KeepLast(1)), std::bind(&ImuAdapter::AtMessageArival, this, std::placeholders::_1));

      velocityXPublisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("imu_velocity_x", 10);
      velocityYPublisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("imu_velocity_y", 10);
      velocityZPublisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("imu_velocity_z", 10);

      orientationOPublisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("imu_orientation_o", 10);
      orientationXPublisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("imu_orientation_x", 10);
      orientationYPublisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("imu_orientation_y", 10);
      orientationZPublisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("imu_orientation_z", 10);

      velocityNoise = std::normal_distribution<float>(0.0,1e-1);
      orientationNoise= std::normal_distribution<float>(0.0,5e-2);
    }

  private:
    void BuildEmptyPacket(std_msgs::msg::Float32MultiArray *message)
    {
      message->layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
            
      message->layout.dim[0].size = 8;
      message->layout.dim[0].stride = 1;
      message->layout.dim[0].label = "smartdata";

      message->data.clear();
      message->data.resize(8); 

      message->data[0] = 0.0;
      message->data[1] = 0.0;
      message->data[2] = 0.0;
      message->data[3] = 0.0;
      message->data[4] = 0.0;
      message->data[5] = 0.0;
      message->data[6] = 0.0;
      message->data[7] = 0.0;
    }

    void AtMessageArival(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
      auto message = std_msgs::msg::Float32MultiArray();
      this->BuildEmptyPacket(&message);
      message.data[6] = msg->angular_velocity.x + velocityNoise(velocityNoiseGenerator);
      velocityXPublisher->publish(message);

      auto messageVelocityY = std_msgs::msg::Float32MultiArray();
      this->BuildEmptyPacket(&messageVelocityY);
      messageVelocityY.data[6] = msg->angular_velocity.y + velocityNoise(velocityNoiseGenerator);
      velocityYPublisher->publish(messageVelocityY);

      auto messageVelocityZ = std_msgs::msg::Float32MultiArray();
      this->BuildEmptyPacket(&messageVelocityZ);
      messageVelocityZ.data[6] = msg->angular_velocity.z + velocityNoise(velocityNoiseGenerator);
      velocityZPublisher->publish(messageVelocityZ);

      

      auto messageOrientationO = std_msgs::msg::Float32MultiArray();
      this->BuildEmptyPacket(&messageOrientationO);
      messageOrientationO.data[6] = msg->orientation.w + orientationNoise(orientationNoiseGenerator);

      auto messageOrientationX = std_msgs::msg::Float32MultiArray();
      this->BuildEmptyPacket(&messageOrientationX);
      messageOrientationX.data[6] = msg->orientation.x + orientationNoise(orientationNoiseGenerator);

      auto messageOrientationY = std_msgs::msg::Float32MultiArray();
      this->BuildEmptyPacket(&messageOrientationY);
      messageOrientationY.data[6] = msg->orientation.y + orientationNoise(orientationNoiseGenerator);

      auto messageOrientationZ = std_msgs::msg::Float32MultiArray();
      this->BuildEmptyPacket(&messageOrientationZ);
      messageOrientationZ.data[6] = msg->orientation.z + orientationNoise(orientationNoiseGenerator);
      
      float orientationNorm = std::sqrt(messageOrientationO.data[6]*messageOrientationO.data[6] + messageOrientationX.data[6]*messageOrientationX.data[6] +
                                messageOrientationY.data[6]*messageOrientationY.data[6] + messageOrientationZ.data[6]*messageOrientationZ.data[6]);

      messageOrientationO.data[6] = messageOrientationO.data[6]/orientationNorm;
      messageOrientationX.data[6] = messageOrientationX.data[6]/orientationNorm;
      messageOrientationY.data[6] = messageOrientationY.data[6]/orientationNorm;
      messageOrientationZ.data[6] = messageOrientationZ.data[6]/orientationNorm;

      orientationOPublisher->publish(messageOrientationO);
      orientationXPublisher->publish(messageOrientationX);
      orientationZPublisher->publish(messageOrientationZ);
      orientationYPublisher->publish(messageOrientationY);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr velocityXPublisher;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr velocityYPublisher;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr velocityZPublisher;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr orientationOPublisher;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr orientationXPublisher;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr orientationYPublisher;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr orientationZPublisher;
    std::default_random_engine velocityNoiseGenerator;
    std::default_random_engine orientationNoiseGenerator;
    std::normal_distribution<float> velocityNoise;
    std::normal_distribution<float> orientationNoise;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuAdapter>());
  rclcpp::shutdown();
  return 0;
}