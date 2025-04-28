#include "motor_controller.h"
#include <iostream>
#include <sstream>

motor_controller::motor_controller()
  : is_connected(false),
  is_moving(false),
  gen(rd()),
  noise_dist(-0.1, 0.1),
  movement_speed(10.0),
  movement_acceleration(5.0),
  position_tolerance(0.001),
  ip_address("127.0.0.1"),
  port(8000),
  device_id(1),
  device_name("Virtual Motor Controller") {

  // Initialize with default position
  current_position = Position6axis();
  target_position = current_position;
}

motor_controller::~motor_controller() {
  // Make sure to disconnect and clean up thread when destroyed
  if (is_connected) {
    disconnect();
  }
}

bool motor_controller::connect(const std::string& ip, int p) {
  if (is_connected) {
    std::cout << "Motor controller is already connected" << std::endl;
    return true;
  }

  ip_address = ip;
  port = p;

  std::cout << "Connecting to motor controller at " << ip_address << ":" << port << "..." << std::endl;

  // Simulate connection delay
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  is_connected = true;

  // Start the position update thread
  position_update_thread = std::thread(&motor_controller::update_position_thread, this);

  std::cout << "Motor controller connected successfully" << std::endl;
  return true;
}

bool motor_controller::disconnect() {
  if (!is_connected) {
    std::cout << "Motor controller is already disconnected" << std::endl;
    return true;
  }

  std::cout << "Disconnecting from motor controller..." << std::endl;

  // Set the flag to terminate the thread
  is_connected = false;

  // Wait for the thread to finish
  if (position_update_thread.joinable()) {
    position_update_thread.join();
  }

  std::cout << "Motor controller disconnected successfully" << std::endl;
  return true;
}

void motor_controller::update_position_thread() {
  const auto update_interval = std::chrono::milliseconds(200);
  double interpolation_step = 0.05 * (movement_speed / 10.0); // Scale by speed setting

  while (is_connected) {
    std::this_thread::sleep_for(update_interval);

    // Lock to safely update the position
    std::lock_guard<std::mutex> lock(position_mutex);

    if (is_moving) {
      // Calculate distance between current and target
      bool reached_target = true;

      // Check X axis
      double diff_x = target_position.x - current_position.x;
      if (std::abs(diff_x) > position_tolerance) {
        reached_target = false;
        current_position.x += diff_x * interpolation_step;
      }

      // Check Y axis
      double diff_y = target_position.y - current_position.y;
      if (std::abs(diff_y) > position_tolerance) {
        reached_target = false;
        current_position.y += diff_y * interpolation_step;
      }

      // Check Z axis
      double diff_z = target_position.z - current_position.z;
      if (std::abs(diff_z) > position_tolerance) {
        reached_target = false;
        current_position.z += diff_z * interpolation_step;
      }

      // Check U axis
      double diff_u = target_position.u - current_position.u;
      if (std::abs(diff_u) > position_tolerance) {
        reached_target = false;
        current_position.u += diff_u * interpolation_step;
      }

      // Check V axis
      double diff_v = target_position.v - current_position.v;
      if (std::abs(diff_v) > position_tolerance) {
        reached_target = false;
        current_position.v += diff_v * interpolation_step;
      }

      // Check W axis
      double diff_w = target_position.w - current_position.w;
      if (std::abs(diff_w) > position_tolerance) {
        reached_target = false;
        current_position.w += diff_w * interpolation_step;
      }

      // Add some noise to simulate real motors
      add_position_noise(current_position, 0.02);

      if (reached_target) {
        is_moving = false;
        std::cout << "Target position reached" << std::endl;
      }
    }
  }
}

bool motor_controller::get_position(Position6axis& out_position) {
  if (!is_connected) {
    std::cout << "Error: Motor controller is not connected" << std::endl;
    return false;
  }

  std::lock_guard<std::mutex> lock(position_mutex);
  out_position = current_position;
  return true;
}

bool motor_controller::move_to_point(const Position6axis& target) {
  if (!is_connected) {
    std::cout << "Error: Motor controller is not connected" << std::endl;
    return false;
  }

  std::lock_guard<std::mutex> lock(position_mutex);
  target_position = target;
  is_moving = true;

  std::cout << "Moving to position: ["
    << target.x << ", "
    << target.y << ", "
    << target.z << ", "
    << target.u << ", "
    << target.v << ", "
    << target.w << "]" << std::endl;

  return true;
}

bool motor_controller::is_controller_connected() const {
  return is_connected;
}

bool motor_controller::is_controller_moving() const {
  return is_moving;
}

void motor_controller::set_movement_parameters(double speed, double acceleration, double tolerance) {
  std::lock_guard<std::mutex> lock(position_mutex);
  movement_speed = speed;
  movement_acceleration = acceleration;
  position_tolerance = tolerance;
}

void motor_controller::set_simulated_position(const Position6axis& position) {
  std::lock_guard<std::mutex> lock(position_mutex);
  current_position = position;
  target_position = position;
  is_moving = false;
}

Position6axis motor_controller::interpolate_position(const Position6axis& start, const Position6axis& end, double t) {
  Position6axis result;

  result.x = start.x + (end.x - start.x) * t;
  result.y = start.y + (end.y - start.y) * t;
  result.z = start.z + (end.z - start.z) * t;
  result.u = start.u + (end.u - start.u) * t;
  result.v = start.v + (end.v - start.v) * t;
  result.w = start.w + (end.w - start.w) * t;

  return result;
}

void motor_controller::add_position_noise(Position6axis& pos, double magnitude) {
  pos.x += noise_dist(gen) * magnitude;
  pos.y += noise_dist(gen) * magnitude;
  pos.z += noise_dist(gen) * magnitude;
  pos.u += noise_dist(gen) * magnitude;
  pos.v += noise_dist(gen) * magnitude;
  pos.w += noise_dist(gen) * magnitude;
}

std::string motor_controller::get_status_json() const {
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
  device["ip"] = ip_address;
  device["port"] = port;
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