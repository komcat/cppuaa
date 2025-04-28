#pragma once
#include "motion/include/motion_types.h"
#include <nlohmann/json.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <functional>
#include <random>
#include <string>

class motor_controller {
private:
  std::thread position_update_thread;
  mutable std::mutex position_mutex;
  std::atomic<bool> is_connected;
  std::atomic<bool> is_moving;
  Position6axis target_position;
  std::random_device rd;
  std::mt19937 gen;
  std::uniform_real_distribution<double> noise_dist;

  // Movement parameters
  double movement_speed;
  double movement_acceleration;
  double position_tolerance;

  // Device information
  std::string ip_address;
  int port;
  int device_id;
  std::string device_name;

  // Internal methods
  void update_position_thread();
  Position6axis interpolate_position(const Position6axis& start, const Position6axis& end, double t);
  void add_position_noise(Position6axis& pos, double magnitude);

public:
  Position6axis current_position;

  motor_controller();
  ~motor_controller();

  // Connection methods
  bool connect(const std::string& ip = "127.0.0.1", int port = 8000);
  bool disconnect();

  // Motion control
  bool get_position(Position6axis& out_position);
  bool move_to_point(const Position6axis& target);
  bool is_controller_connected() const;
  bool is_controller_moving() const;

  // Configuration
  void set_movement_parameters(double speed, double acceleration, double tolerance);

  // For simulation and testing
  void set_simulated_position(const Position6axis& position);
  std::string get_status_json() const;
};