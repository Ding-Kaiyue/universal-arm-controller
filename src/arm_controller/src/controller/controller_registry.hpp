#ifndef __CONTROLLER_REGISTRY_HPP__
#define __CONTROLLER_REGISTRY_HPP__

#include "controller_interface.hpp"

std::unordered_map<std::string, ControllerInterface::Creator> get_available_controllers();



#endif      // __CONTROLLER_REGISTRY_HPP__
