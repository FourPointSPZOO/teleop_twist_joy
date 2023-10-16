// Copyright Haul Vision 2023

#ifndef TELEOP_TWIST_JOY_L2PING_HPP
#define TELEOP_TWIST_JOY_L2PING_HPP

#include <boost/process.hpp>
#include <regex>
#include <string>
#include <fmt/format.h>

namespace bluetooth {
    bool ping_device(const std::string &address, double timeout);
}

#endif //TELEOP_TWIST_JOY_L2PING_HPP
