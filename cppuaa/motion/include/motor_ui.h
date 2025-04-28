#pragma once

#include "imgui.h"
#include "IMotorController.h"
#include "MotorControllerFactory.h"
#include <memory>
#include <string>

// Original functions
void initialize_motor_controller(MotorControllerType type = MotorControllerType::VIRTUAL);
void initialize_motor_controller_by_name(const std::string& type_name);
void cleanup_motor_controller();
std::shared_ptr<IMotorController> get_motor_controller();
MotorControllerType get_controller_type();
bool switch_controller_type(MotorControllerType type);
void show_motor_controller_window();
void print_position(const Position6axis& pos);

// New functions for multi-controller support
void initialize_controllers(); // Function to set up 3 PI controllers and 1 ACS controller
bool switch_controller(int index);
void connect_all_controllers(const std::string& ip_base = "192.168.1.", int start_port = 8000);
void disconnect_all_controllers();
bool connect_controller(int id, const std::string& ip, int port);
// Add this to motor_ui.h
bool connect_controller(int id, const std::string& ip, int port, MotorControllerType type);

// Add to motor_ui.h
void set_verbose_logging(bool verbose);