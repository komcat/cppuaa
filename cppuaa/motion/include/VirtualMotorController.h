#pragma once
#include "IMotorController.h"
#include <nlohmann/json.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <functional>
#include <random>
#include <string>

class VirtualMotorController : public IMotorController {
private:
  std::thread position_update_thread;
  mutable std::mutex position_mutex;
  std::atomic<bool> is_connected;
  std::atomic<bool> is_moving;
  Position6axis current_position;
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
  VirtualMotorController();
  ~VirtualMotorController() override;

  // Connection methods
  bool connect(const std::string& ip = "127.0.0.1", int port = 8000) override;
  bool disconnect() override;
  bool is_controller_connected() const override;

  // Motion control
  bool get_position(Position6axis& out_position) override;
  bool move_to_point(const Position6axis& target) override;
  bool is_controller_moving() const override;

  // Configuration
  void set_movement_parameters(double speed, double acceleration, double tolerance) override;

  // Status reporting
  std::string get_status_json() const override;
  std::string get_controller_type() const override { return "VIRTUAL"; }

  // Methods specific to virtual controller
  void set_simulated_position(const Position6axis& position);
};