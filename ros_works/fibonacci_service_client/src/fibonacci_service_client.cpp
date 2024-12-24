#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <iostream> // Для пользовательского ввода

class FibonacciServiceClient : public rclcpp::Node {
public:
    FibonacciServiceClient() : Node("fibonacci_service_client") {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("fibonacci_service");

        // Ждем, пока сервер будет доступен
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for the Fibonacci service to become available...");
        }

        int64_t index;
        RCLCPP_INFO(this->get_logger(), "Enter the index of the Fibonacci number:");
        std::cin >> index;

        send_request(index);
    }

private:
    void send_request(int64_t index) {
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = index;

        auto future = client_->async_send_request(request);
        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Fibonacci number at index %ld: %ld", index, response->sum);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FibonacciServiceClient>());
    rclcpp::shutdown();
    return 0;
}
