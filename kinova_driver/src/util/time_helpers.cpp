#include <sstream>

std::string nanoseconds_to_string(double nanoseconds)
{
    const auto seconds = static_cast<int>(nanoseconds * 1e-9);
    double left_nanoseconds = nanoseconds - seconds * 1e-9;
    const auto milliseconds = static_cast<int>(left_nanoseconds * 1e-6);
    left_nanoseconds -= milliseconds * 1e6;
    const auto microseconds = static_cast<int>(left_nanoseconds * 1e-3);
    left_nanoseconds -= microseconds * 1e3;

    std::stringstream ss;
    ss << seconds << "s " << milliseconds << "ms " << microseconds << "us " << left_nanoseconds << "ns";
    return ss.str();
}
