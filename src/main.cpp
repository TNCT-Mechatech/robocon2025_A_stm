#include <rclcpp/rclcpp.hpp>

#include <SerialBridge.hpp>
//  serial driver for linux/ros
#include <LinuxHardwareSerial.hpp>
#include "ArmStruct.hpp"
#include "EncoderStruct.hpp"
#include "Manipulator3Dof.hpp"

using namespace std::chrono_literals;

// 要変更
#define SERIAL_PATH "/dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.2" // 電源側USB3.0

class MasterClass : public rclcpp::Node
{
public:
    MasterClass(const char port[], speed_t baud_rate = B9600)
        : Node("master_class"),
          serial_dev(new LinuxHardwareSerial(port, baud_rate)),
          serial(new SerialBridge(serial_dev))
    {
        // -- 送信 --

        // 左右アーム
        serial->add_frame(0, &ass);

        // -- 受信 --
        // 座標
        serial.add_frame(1, &ess);

        // joy subscription
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "arm/joy", 10, std::bind(&MasterClass::joy_callback, this, _1));

        timer_ = this->create_wall_timer(10ms, std::bind(&MasterClass::on_timer, this));
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
    }

    void on_timer()
    {
        auto now = this->now();
        serial->update();
        if (ess.was_updated())
        {
        }
        last = now;
    }

private:
    auto last = this->now();
    SerialDev *serial_dev;
    SerialBridge *serial;
    ArmStruct ass;
    EncoderStruct ess;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MasterClass>(SERIAL_PATH);
    rclcpp::executors::MultithreadExecuter exec(rclcpp::ExecutorOptions(), 2);

    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}