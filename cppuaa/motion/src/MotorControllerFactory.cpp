#include "MotorControllerFactory.h"
#include "VirtualMotorController.h"
#include "PIMotorController.h"
#include "ACSMotorController.h"
#include <iostream>
#include <algorithm>
#include <cctype>

std::shared_ptr<IMotorController> MotorControllerFactory::create_controller(MotorControllerType type) {
  switch (type) {
  case MotorControllerType::VIRTUAL:
    std::cout << "Creating Virtual Motor Controller" << std::endl;
    return std::make_shared<VirtualMotorController>();

  case MotorControllerType::PI:
    std::cout << "Creating PI Motor Controller" << std::endl;
    return std::make_shared<PIMotorController>();

  case MotorControllerType::ACS:
    std::cout << "Creating ACS Motor Controller" << std::endl;
    return std::make_shared<ACSMotorController>();

  default:
    std::cerr << "Unknown controller type, defaulting to Virtual" << std::endl;
    return std::make_shared<VirtualMotorController>();
  }
}

// Helper function to convert string to uppercase
std::string to_upper(const std::string& input) {
  std::string result = input;
  std::transform(result.begin(), result.end(), result.begin(),
    [](unsigned char c) { return std::toupper(c); });
  return result;
}

std::shared_ptr<IMotorController> MotorControllerFactory::create_controller_from_string(const std::string& type_name) {
  std::string type_upper = to_upper(type_name);

  if (type_upper == "VIRTUAL" || type_upper == "SIMULATION" || type_upper == "SIM") {
    return create_controller(MotorControllerType::VIRTUAL);
  }
  else if (type_upper == "PI" || type_upper == "PHYSIK INSTRUMENTE") {
    return create_controller(MotorControllerType::PI);
  }
  else if (type_upper == "ACS" || type_upper == "ADVANCED CONTROL SYSTEMS") {
    return create_controller(MotorControllerType::ACS);
  }
  else {
    std::cerr << "Unknown controller type name '" << type_name << "', defaulting to Virtual" << std::endl;
    return create_controller(MotorControllerType::VIRTUAL);
  }
}