#include "ACSMotorController.h"
#include <iostream>
#include <sstream>
#include <random>
#include <thread>

ACSMotorController::ACSMotorController()
  : is_connected(false),
  is_moving(false),
  movement_speed(10.0),
  movement_acceleration(5.0),
  position_tolerance(0.001),
  ip_address("127.0.0.1"),
  port(8000),
  device_id(1),
  device_name("ACS Motion Controller"),
  handle_id(-1),
  firmware_version("SPiiPlus 2.70") {

  // Initialize with default position
  current_position = Position6axis();
  target_position = current_position;

  // Initialize axis handles (would be real axis references in actual implementation)
  axis_handles.resize(6, -1);
}

ACSMotorController::~ACSMotorController() {
  // Make sure to disconnect and clean up thread when destroyed
  if (is_connected) {
    disconnect();
  }
}

void ACSMotorController::simulate_command_delay() {
  // Simulate network and command processing delays
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> delay_dist(50, 150);
  std::this_thread::sleep_for(std::chrono::milliseconds(delay_dist(gen)));
}

bool ACSMotorController::connect(const std::string& ip, int p) {
  if (is_connected) {
    std::cout << "[ACS] Motor controller is already connected" << std::endl;
    return true;
  }

  ip_address = ip;
  port = p;

  std::cout << "[ACS] Connecting to ACS SPiiPlus controller at " << ip_address << ":" << port << "..." << std::endl;

  // Simulate connection process - in a real implementation, would connect to actual hardware
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));

  // Simulate controller response
  std::cout << "[ACS] Controller detected: SPiiPlus, Firmware v" << firmware_version << std::endl;

  // Initialize axis handles (would be actual hardware axes in real implementation)
  for (int i = 0; i < 6; i++) {
    axis_handles[i] = i + 1;
    std::cout << "[ACS] Initialized axis " << i + 1 << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Simulate successful connection
  handle_id = 12345;  // Dummy handle ID
  is_connected = true;

  // Start the position update thread
  position_update_thread = std::thread(&ACSMotorController::update_position_thread, this);

  std::cout << "[ACS] Motor controller connected successfully" << std::endl;
  return true;
}

bool ACSMotorController::disconnect() {
  if (!is_connected) {
    std::cout << "[ACS] Motor controller is already disconnected" << std::endl;
    return true;
  }

  std::cout << "[ACS] Disconnecting from ACS controller..." << std::endl;

  // Set the flag to terminate the thread
  is_connected = false;

  // Wait for the thread to finish
  if (position_update_thread.joinable()) {
    position_update_thread.join();
  }

  // In a real implementation, would close the connection to hardware
  for (int i = 0; i < 6; i++) {
    axis_handles[i] = -1;
    std::cout << "[ACS] Released axis " << i + 1 << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  handle_id = -1;

  std::cout << "[ACS] Motor controller disconnected successfully" << std::endl;
  return true;
}

void ACSMotorController::update_position_thread() {
  const auto update_interval = std::chrono::milliseconds(50);  // ACS controllers typically have faster update rates

  while (is_connected) {
    std::this_thread::sleep_for(update_interval);

    // Lock to safely update the position
    std::lock_guard<std::mutex> lock(position_mutex);

    if (is_moving) {
      // In a real implementation, this would query the actual hardware for position
      // For ACS, simulate a more precise and controlled movement model

      bool reached_target = true;

      // ACS controllers typically use a trapezoidal motion profile
      // We'll simulate this with a different algorithm than the virtual controller
      double step_factor = 0.2 * (movement_speed / 10.0);

      // Current velocity - would come from actual hardware in real implementation
      static double vel_x = 0, vel_y = 0, vel_z = 0, vel_u = 0, vel_v = 0, vel_w = 0;
      double accel_factor = movement_acceleration / 20.0;

      // X-axis motion
      double diff_x = target_position.x - current_position.x;
      double target_vel_x = diff_x > 0 ? movement_speed : -movement_speed;

      // Apply acceleration limits
      if (std::abs(diff_x) < std::abs(vel_x) * vel_x / (2 * accel_factor)) {
        // Deceleration phase
        vel_x = vel_x > 0 ? std::max(vel_x - accel_factor, 0.0) : std::min(vel_x + accel_factor, 0.0);
      }
      else if (std::abs(vel_x) < std::abs(target_vel_x)) {
        // Acceleration phase
        vel_x = vel_x > 0 ? std::min(vel_x + accel_factor, target_vel_x) : std::max(vel_x - accel_factor, target_vel_x);
      }

      if (std::abs(diff_x) > position_tolerance) {
        reached_target = false;
        current_position.x += vel_x * 0.05;  // 50ms update interval
      }
      else {
        current_position.x = target_position.x;
        vel_x = 0;
      }

      // Y-axis motion (similar pattern)
      double diff_y = target_position.y - current_position.y;
      double target_vel_y = diff_y > 0 ? movement_speed : -movement_speed;

      if (std::abs(diff_y) < std::abs(vel_y) * vel_y / (2 * accel_factor)) {
        vel_y = vel_y > 0 ? std::max(vel_y - accel_factor, 0.0) : std::min(vel_y + accel_factor, 0.0);
      }
      else if (std::abs(vel_y) < std::abs(target_vel_y)) {
        vel_y = vel_y > 0 ? std::min(vel_y + accel_factor, target_vel_y) : std::max(vel_y - accel_factor, target_vel_y);
      }

      if (std::abs(diff_y) > position_tolerance) {
        reached_target = false;
        current_position.y += vel_y * 0.05;
      }
      else {
        current_position.y = target_position.y;
        vel_y = 0;
      }

      // Z-axis motion
      double diff_z = target_position.z - current_position.z;
      double target_vel_z = diff_z > 0 ? movement_speed : -movement_speed;

      if (std::abs(diff_z) < std::abs(vel_z) * vel_z / (2 * accel_factor)) {
        vel_z = vel_z > 0 ? std::max(vel_z - accel_factor, 0.0) : std::min(vel_z + accel_factor, 0.0);
      }
      else if (std::abs(vel_z) < std::abs(target_vel_z)) {
        vel_z = vel_z > 0 ? std::min(vel_z + accel_factor, target_vel_z) : std::max(vel_z - accel_factor, target_vel_z);
      }

      if (std::abs(diff_z) > position_tolerance) {
        reached_target = false;
        current_position.z += vel_z * 0.05;
      }
      else {
        current_position.z = target_position.z;
        vel_z = 0;
      }

      // U-axis motion (rotation)
      double diff_u = target_position.u - current_position.u;
      double target_vel_u = diff_u > 0 ? movement_speed / 2 : -movement_speed / 2;  // Slower for rotations

      if (std::abs(diff_u) < std::abs(vel_u) * vel_u / (2 * accel_factor)) {
        vel_u = vel_u > 0 ? std::max(vel_u - accel_factor, 0.0) : std::min(vel_u + accel_factor, 0.0);
      }
      else if (std::abs(vel_u) < std::abs(target_vel_u)) {
        vel_u = vel_u > 0 ? std::min(vel_u + accel_factor, target_vel_u) : std::max(vel_u - accel_factor, target_vel_u);
      }

      if (std::abs(diff_u) > position_tolerance) {
        reached_target = false;
        current_position.u += vel_u * 0.05;
      }
      else {
        current_position.u = target_position.u;
        vel_u = 0;
      }

      // V-axis motion (rotation)
      double diff_v = target_position.v - current_position.v;
      double target_vel_v = diff_v > 0 ? movement_speed / 2 : -movement_speed / 2;

      if (std::abs(diff_v) < std::abs(vel_v) * vel_v / (2 * accel_factor)) {
        vel_v = vel_v > 0 ? std::max(vel_v - accel_factor, 0.0) : std::min(vel_v + accel_factor, 0.0);
      }
      else if (std::abs(vel_v) < std::abs(target_vel_v)) {
        vel_v = vel_v > 0 ? std::min(vel_v + accel_factor, target_vel_v) : std::max(vel_v - accel_factor, target_vel_v);
      }

      if (std::abs(diff_v) > position_tolerance) {
        reached_target = false;
        current_position.v += vel_v * 0.05;
      }
      else {
        current_position.v = target_position.v;
        vel_v = 0;
      }

      // W-axis motion (rotation)
      double diff_w = target_position.w - current_position.w;
      double target_vel_w = diff_w > 0 ? movement_speed / 2 : -movement_speed / 2;

      if (std::abs(diff_w) < std::abs(vel_w) * vel_w / (2 * accel_factor)) {
        vel_w = vel_w > 0 ? std::max(vel_w - accel_factor, 0.0) : std::min(vel_w + accel_factor, 0.0);
      }
      else if (std::abs(vel_w) < std::abs(target_vel_w)) {
        vel_w = vel_w > 0 ? std::min(vel_w + accel_factor, target_vel_w) : std::max(vel_w - accel_factor, target_vel_w);
      }

      if (std::abs(diff_w) > position_tolerance) {
        reached_target = false;
        current_position.w += vel_w * 0.05;
      }
      else {
        current_position.w = target_position.w;
        vel_w = 0;
      }

      if (reached_target) {
        is_moving = false;
        std::cout << "[ACS] Target position reached with high precision" << std::endl;
      }
    }
  }
}

bool ACSMotorController::check_error(const std::string& operation) {
  // In a real implementation, this would check for hardware errors
  // For simulation, randomly generate errors very rarely
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> error_dist(1, 100);

  if (error_dist(gen) == 1) {  // 1% chance of error
    last_error = "Simulated error during " + operation;
    std::cerr << "[ACS] ERROR: " << last_error << std::endl;
    return false;
  }

  return true;
}

bool ACSMotorController::get_position(Position6axis& out_position) {
  if (!is_connected) {
    std::cout << "[ACS] Error: Motor controller is not connected" << std::endl;
    return false;
  }

  // Simulate command delay
  simulate_command_delay();

  // In real implementation, would query hardware for actual position
  std::lock_guard<std::mutex> lock(position_mutex);
  out_position = current_position;

  return check_error("get_position");
}

bool ACSMotorController::move_to_point(const Position6axis& target) {
  if (!is_connected) {
    std::cout << "[ACS] Error: Motor controller is not connected" << std::endl;
    return false;
  }

  // Simulate command delay
  simulate_command_delay();

  // In real implementation, would send command to hardware
  std::lock_guard<std::mutex> lock(position_mutex);
  target_position = target;
  is_moving = true;

  //std::cout << "[ACS] Moving to position: ["
  //  << target.x << ", "
  //  << target.y << ", "
  //  << target.z << ", "
  //  << target.u << ", "
  //  << target.v << ", "
  //  << target.w << "]" << std::endl;

  return check_error("move_to_point");
}

bool ACSMotorController::is_controller_connected() const {
  return is_connected;
}

bool ACSMotorController::is_controller_moving() const {
  return is_moving;
}

void ACSMotorController::set_movement_parameters(double speed, double acceleration, double tolerance) {
  std::lock_guard<std::mutex> lock(position_mutex);
  movement_speed = speed;
  movement_acceleration = acceleration;
  position_tolerance = tolerance;

  // In real implementation, would send these parameters to the hardware
  std::cout << "[ACS] Set movement parameters: speed=" << speed
    << ", acceleration=" << acceleration
    << ", tolerance=" << tolerance << std::endl;

  // Simulate command delay
  simulate_command_delay();
}

std::string ACSMotorController::get_firmware_version() const {
  return firmware_version;
}

std::string ACSMotorController::get_last_error() const {
  return last_error;
}

bool ACSMotorController::emergency_stop() {
  if (!is_connected) {
    std::cout << "[ACS] Error: Motor controller is not connected" << std::endl;
    return false;
  }

  std::cout << "[ACS] !!! EMERGENCY STOP !!! Halting all axes immediately" << std::endl;

  // In real implementation, would send emergency stop command to hardware
  std::lock_guard<std::mutex> lock(position_mutex);
  is_moving = false;

  // Simulate immediate stop - in real hardware this would happen through hardware signals
  simulate_command_delay();

  return check_error("emergency_stop");
}

std::string ACSMotorController::get_status_json() const {
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
  device["type"] = "ACS";
  device["ip"] = ip_address;
  device["port"] = port;
  device["handle_id"] = handle_id;
  device["firmware_version"] = firmware_version;

  // ACS specific info
  nlohmann::json axes;
  for (size_t i = 0; i < axis_handles.size(); i++) {
    axes[std::to_string(i + 1)] = axis_handles[i];
  }
  device["axes"] = axes;

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