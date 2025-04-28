#pragma once
#include "motion/include/motion_types.h"
#include <string>

class IMotorController {
public:
  // Virtual destructor for proper cleanup
  virtual ~IMotorController() = default;

  // Connection methods
  virtual bool connect(const std::string& ip = "127.0.0.1", int port = 8000) = 0;
  virtual bool disconnect() = 0;
  virtual bool is_controller_connected() const = 0;

  // Motion control
  virtual bool get_position(Position6axis& out_position) = 0;
  virtual bool move_to_point(const Position6axis& target) = 0;
  virtual bool is_controller_moving() const = 0;

  // Configuration
  virtual void set_movement_parameters(double speed, double acceleration, double tolerance) = 0;

  // Status reporting
  virtual std::string get_status_json() const = 0;
  virtual std::string get_controller_type() const = 0;
};