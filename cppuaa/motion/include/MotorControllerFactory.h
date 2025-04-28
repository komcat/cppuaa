#pragma once
#include "IMotorController.h"
#include <memory>
#include <string>

enum class MotorControllerType {
  VIRTUAL,
  PI,  // PI controller
  ACS  // ACS controller
};

class MotorControllerFactory {
public:
  static std::shared_ptr<IMotorController> create_controller(MotorControllerType type);
  static std::shared_ptr<IMotorController> create_controller_from_string(const std::string& type_name);
};