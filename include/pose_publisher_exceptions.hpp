#include <rclcpp/rclcpp.hpp>
#include <exception>

class InvalidDataTypeException : public std::exception
{
public:
    InvalidDataTypeException(const std::string& message) : message_(message) {}

    const char* what() const noexcept override
    {
        return message_.c_str();
    }

private:
    std::string message_;
};