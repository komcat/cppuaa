#pragma once
#include "IMotorController.h"
#include <nlohmann/json.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <string>

class PIMotorController : public IMotorController {
private:
  std::thread position_update_thread;
  mutable std::mutex position_mutex;
  std::atomic<bool> is_connected;
  std::atomic<bool> is_moving;
  Position6axis current_position;
  Position6axis target_position;

  // Movement parameters
  double movement_speed;
  double movement_acceleration;
  double position_tolerance;

  // Device information
  std::string ip_address;
  int port;
  int device_id;
  std::string device_name;

  // PI-specific parameters
  int socket_id;
  std::string last_error;

  // Internal methods
  void update_position_thread();
  bool check_error(const std::string& operation);

public:
  PIMotorController();
  ~PIMotorController() override;

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
  std::string get_controller_type() const override { return "PI"; }

  // PI-specific methods
  std::string get_last_error() const;
};