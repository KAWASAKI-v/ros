#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

class FibonacciServiceServer : public rclcpp::Node {
public:
    FibonacciServiceServer() : Node("fibonacci_service_server") {
        // Создаем сервер
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "fibonacci_service", 
            std::bind(&FibonacciServiceServer::handle_request, this, std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(this->get_logger(), "Service 'fibonacci_service' is ready.");
    }

private:
    void handle_request(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response
    ) {
        int64_t n = request->a;  // Индекс числа Фибоначчи
        response->sum = calculate_fibonacci(n);
        RCLCPP_INFO(this->get_logger(), "Request for index %ld -> Fibonacci: %ld", n, response->sum);
    }

    int64_t calculate_fibonacci(int64_t n) {
        if (n <= 0) return 0;
        if (n == 1) return 1;

        int64_t prev = 0, curr = 1;
        for (int64_t i = 2; i <= n; ++i) {
            int64_t next = prev + curr;
            prev = curr;
            curr = next;
        }
        return curr;
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FibonacciServiceServer>());
    rclcpp::shutdown();
    return 0;
}
