#include "PIMotorController.h"
#include <iostream>
#include <sstream>

PIMotorController::PIMotorController()
  : is_connected(false),
  is_moving(false),
  movement_speed(10.0),
  movement_acceleration(5.0),
  position_tolerance(0.001),
  ip_address("127.0.0.1"),
  port(8000),
  device_id(1),
  device_name("PI Motor Controller"),
  socket_id(-1) {

  // Initialize with default position
  current_position = Position6axis();
  target_position = current_position;
}

PIMotorController::~PIMotorController() {
  // Make sure to disconnect and clean up thread when destroyed
  if (is_connected) {
    disconnect();
  }
}

bool PIMotorController::connect(const std::string& ip, int p) {
  if (is_connected) {
    std::cout << "[PI] Motor controller is already connected" << std::endl;
    return true;
  }

  ip_address = ip;
  port = p;

  std::cout << "[PI] Connecting to motor controller at " << ip_address << ":" << port << "..." << std::endl;

  // Simulate connection delay - in a real implementation, this would connect to actual hardware
  std::this_thread::sleep_for(std::chrono::milliseconds(800));

  // Simulate successful connection - in real implementation, would check for hardware response
  socket_id = 1;  // Dummy socket ID
  is_connected = true;

  // Start the position update thread
  position_update_thread = std::thread(&PIMotorController::update_position_thread, this);

  std::cout << "[PI] Motor controller connected successfully" << std::endl;
  return true;
}

bool PIMotorController::disconnect() {
  if (!is_connected) {
    std::cout << "[PI] Motor controller is already disconnected" << std::endl;
    return true;
  }

  std::cout << "[PI] Disconnecting from motor controller..." << std::endl;

  // Set the flag to terminate the thread
  is_connected = false;

  // Wait for the thread to finish
  if (position_update_thread.joinable()) {
    position_update_thread.join();
  }

  // Close the socket in real implementation
  socket_id = -1;

  std::cout << "[PI] Motor controller disconnected successfully" << std::endl;
  return true;
}

void PIMotorController::update_position_thread() {
  const auto update_interval = std::chrono::milliseconds(100);  // PI controllers typically have faster update rates

  while (is_connected) {
    std::this_thread::sleep_for(update_interval);

    // Lock to safely update the position
    std::lock_guard<std::mutex> lock(position_mutex);

    if (is_moving) {
      // In a real implementation, this would query the actual hardware for position
      // For now, we'll simulate a move with more abrupt movements (less smoothing than virtual)

      bool reached_target = true;
      double step_factor = 0.1 * (movement_speed / 10.0);

      // Update position for each axis
      double diff_x = target_position.x - current_position.x;
      if (std::abs(diff_x) > position_tolerance) {
        reached_target = false;
        current_position.x += diff_x * step_factor;
      }
      else {
        current_position.x = target_position.x;  // Snap to exact position when close
      }

      double diff_y = target_position.y - current_position.y;
      if (std::abs(diff_y) > position_tolerance) {
        reached_target = false;
        current_position.y += diff_y * step_factor;
      }
      else {
        current_position.y = target_position.y;
      }

      double diff_z = target_position.z - current_position.z;
      if (std::abs(diff_z) > position_tolerance) {
        reached_target = false;
        current_position.z += diff_z * step_factor;
      }
      else {
        current_position.z = target_position.z;
      }

      double diff_u = target_position.u - current_position.u;
      if (std::abs(diff_u) > position_tolerance) {
        reached_target = false;
        current_position.u += diff_u * step_factor;
      }
      else {
        current_position.u = target_position.u;
      }

      double diff_v = target_position.v - current_position.v;
      if (std::abs(diff_v) > position_tolerance) {
        reached_target = false;
        current_position.v += diff_v * step_factor;
      }
      else {
        current_position.v = target_position.v;
      }

      double diff_w = target_position.w - current_position.w;
      if (std::abs(diff_w) > position_tolerance) {
        reached_target = false;
        current_position.w += diff_w * step_factor;
      }
      else {
        current_position.w = target_position.w;
      }

      if (reached_target) {
        is_moving = false;
        std::cout << "[PI] Target position reached" << std::endl;
      }
    }
  }
}

bool PIMotorController::check_error(const std::string& operation) {
  // In a real implementation, this would check for hardware errors
  // For now, always return success
  return true;
}

bool PIMotorController::get_position(Position6axis& out_position) {
  if (!is_connected) {
    std::cout << "[PI] Error: Motor controller is not connected" << std::endl;
    return false;
  }

  // In real implementation, would query hardware for actual position
  std::lock_guard<std::mutex> lock(position_mutex);
  out_position = current_position;
  return true;
}

bool PIMotorController::move_to_point(const Position6axis& target) {
  if (!is_connected) {
    std::cout << "[PI] Error: Motor controller is not connected" << std::endl;
    return false;
  }

  // In real implementation, would send command to hardware
  std::lock_guard<std::mutex> lock(position_mutex);
  target_position = target;
  is_moving = true;

  //std::cout << "[PI] Moving to position: ["
  //  << target.x << ", "
  //  << target.y << ", "
  //  << target.z << ", "
  //  << target.u << ", "
  //  << target.v << ", "
  //  << target.w << "]" << std::endl;

  return check_error("move_to_point");
}

bool PIMotorController::is_controller_connected() const {
  return is_connected;
}

bool PIMotorController::is_controller_moving() const {
  return is_moving;
}

void PIMotorController::set_movement_parameters(double speed, double acceleration, double tolerance) {
  std::lock_guard<std::mutex> lock(position_mutex);
  movement_speed = speed;
  movement_acceleration = acceleration;
  position_tolerance = tolerance;

  // In real implementation, would send these parameters to the hardware
  std::cout << "[PI] Set movement parameters: speed=" << speed
    << ", acceleration=" << acceleration
    << ", tolerance=" << tolerance << std::endl;
}

std::string PIMotorController::get_last_error() const {
  return last_error;
}

std::string PIMotorController::get_status_json() const {
  // Using mutable mutex to lock in a const method
  std::lock_guard<std::mutex> lock(position_mutex);

  // Create a JSON representation of the controller state
  nlohmann::json status;

  // Device status
  status["connected"] = is_connected.load();
  status["moving"] = is_moving.load();

  // Device info
  nlohmann::json device;
  device["id"] = device_id;
  device["name"] = device_name;
  device["type"] = "PI";
  device["ip"] = ip_address;
  device["port"] = port;
  device["socket_id"] = socket_id;
  status["device"] = device;

  // Current position
  nlohmann::json position;
  position["x"] = current_position.x;
  position["y"] = current_position.y;
  position["z"] = current_position.z;
  position["u"] = current_position.u;
  position["v"] = current_position.v;
  position["w"] = current_position.w;
  status["position"] = position;

  // Target position
  nlohmann::json target;
  target["x"] = target_position.x;
  target["y"] = target_position.y;
  target["z"] = target_position.z;
  target["u"] = target_position.u;
  target["v"] = target_position.v;
  target["w"] = target_position.w;
  status["target"] = target;

  // Movement parameters
  nlohmann::json parameters;
  parameters["speed"] = movement_speed;
  parameters["acceleration"] = movement_acceleration;
  parameters["tolerance"] = position_tolerance;
  status["parameters"] = parameters;

  // Return formatted JSON string
  return status.dump(2);
}