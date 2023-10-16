// Copyright Haul Vision 2023

#include "teleop_twist_joy/l2ping.hpp"

namespace bp = boost::process;


bool bluetooth::ping_device(const std::string &address, double timeout) {
    std::ostringstream os;
    bp::ipstream is_out;
    bp::ipstream is_err;

    bp::child c(
            fmt::format("l2ping -c 1 -t {} {}", timeout, address),
            bp::std_out > is_out,
            bp::std_err > is_err    // stderr can be used for debug
    );

    /*
     Output example of `l2ping`

     Ping: AA:BB:CC:DD:EE:FF from 01:23:45:67:89:01 (data size 44) ...
     44 bytes from AA:BB:CC:DD:EE:FF id 0 time 23.08ms
     1 sent, 1 received, 0% loss
    */
    std::string line;
    // Skip header
    std::getline(is_out, line);
    // Get 2nd line
    std::getline(is_out, line);

    std::cmatch match;

    // Echo response received
    const std::regex filter_time(R"(.*time (\d+\.\d+)ms)");
    if (std::regex_match(line.c_str(), match, filter_time)) {
        return true;
    }

    return false;
}
