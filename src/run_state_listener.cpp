
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class RunStateListener : public rclcpp::Node {
  public:
    RunStateListener(double timeout_sec, std::ostream &writer,
                     std::optional<std::ofstream> &&log_file)
        : Node("run_state_listener"), writer_(writer),
          log_file_(std::move(log_file)),
          timeout_(std::chrono::duration<double>(timeout_sec)),
          last_msg_time_(std::chrono::steady_clock::now()) {
        sub_ = create_subscription<std_msgs::msg::Bool>(
            "/run_state", rclcpp::QoS(1),
            std::bind(&RunStateListener::callback, this,
                      std::placeholders::_1));

        timer_ = create_wall_timer(
            500ms, std::bind(&RunStateListener::watchdog, this));
    }

  private:
    void callback(const std_msgs::msg::Bool::SharedPtr msg) {
        last_msg_time_ = std::chrono::steady_clock::now();
        emit_text(msg->data ? "true" : "false");
    }

    void watchdog() {
        if (last_msg_time_ == unreachable_) {
            return;
        }
        auto now = std::chrono::steady_clock::now();
        if ((now - last_msg_time_) > timeout_) {
            emit_text("unavailable");
            last_msg_time_ = unreachable_;
        }
    }

    void emit_text(const std::string &text) {
        writer_ << text << '\n';
        writer_.flush();
        if (log_file_) {
            *log_file_ << text << '\n';
            log_file_->flush();
        }
    }

    std::ostream &writer_;
    std::optional<std::ofstream> log_file_;
    const std::chrono::duration<double> timeout_;
    std::chrono::steady_clock::time_point last_msg_time_;
    const std::chrono::steady_clock::time_point unreachable_{};

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

static void usage() {
    std::cerr << "Usage: run_state_listener [--unavailable-timeout SEC] "
                 "[--log-file FILE]\n";
}

int main(int argc, char *argv[]) {
    double timeout_sec = 5.0;
    std::optional<std::string> log_path;
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--unavailable-timeout" && i + 1 < argc) {
            timeout_sec = std::stod(argv[++i]);
        } else if (arg == "--log-file" && i + 1 < argc) {
            log_path = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            usage();
            return 0;
        } else {
            break;
        }
    }

    rclcpp::init(argc, argv);

    std::optional<std::ofstream> log_file;
    if (log_path) {
        log_file.emplace(*log_path, std::ios::out | std::ios::trunc);
        if (!*log_file) {
            std::cerr << "Cannot open " << *log_path << " for writing\n";
            return 1;
        }
    }

    auto node = std::make_shared<RunStateListener>(timeout_sec, std::cout,
                                                   std::move(log_file));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
