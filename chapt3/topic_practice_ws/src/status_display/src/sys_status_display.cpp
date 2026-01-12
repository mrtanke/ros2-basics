#include <QApplication>
#include <QLabel>
#include <QString>
#include <rclcpp/rclcpp.hpp>
#include <status_interfaces/msg/system_status.hpp>

using SystemStatus = status_interfaces::msg::SystemStatus;

class SysStatusDisplay : public rclcpp::Node
{
private:
    /* data */
    rclcpp::Subscription<SystemStatus>::SharedPtr subscriber_;
    QLabel *label_;

public:
    SysStatusDisplay(/* args */):Node("sys_status_display") {
        label_ = new QLabel();
        subscriber_ = this->create_subscription<SystemStatus>(
            "sys_status", 10,
            [&](const SystemStatus::SharedPtr msg)->void{
                label_->setText(get_qstr_from_msg(msg));
            }
        );
        label_->setText(get_qstr_from_msg(std::make_shared<SystemStatus>()));
        label_->show();

    };

    QString get_qstr_from_msg(const SystemStatus::SharedPtr msg) {
        std::stringstream show_str;
        show_str << "======System Status Visulazation Tool======\n"
        << "Data time: \t" << msg->stamp.sec << "\ts\n" 
        << "Host Name: \t" << msg->host_name << "\t\n"
        << "CPU Usage: \t" << msg->cpu_percent << "\t%\n"
        << "Memory Usage: \t" << msg->memory_percent << "\t%\n"
        << "Total Memory: \t" << msg->memory_total << "\tMB\n"
        << "Available Memory: " << msg->memory_available << "\tMB\n"
        << "Network Sent: \t" << msg->net_sent << "\tBytes\n"
        << "Network Received: " << msg->net_recv << "\tBytes\n"
        << "===========================================";
        
        return QString::fromStdString(show_str.str());
    }
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    auto node = std::make_shared<SysStatusDisplay>();

    std::thread spin_thread([&]()->void{
        rclcpp::spin(node); // block here
    });
    
    spin_thread.detach();
    app.exec(); // exec the qt app, block here

    return 0;
}