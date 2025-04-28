// Modified motor_ui.cpp to support multiple controllers

#include "motor_ui.h"
#include "VirtualMotorController.h"
#include "PIMotorController.h"
#include "ACSMotorController.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <vector>

// Global controller instances
struct ControllerInstance {
  std::shared_ptr<IMotorController> controller;
  MotorControllerType type;
  int id;
  std::string name;
};

static std::vector<ControllerInstance> g_controllers;
static int g_active_controller_index = 0;
// Add this at the top of motor_ui.cpp with your other globals
static bool g_verbose_logging = false;


// Then modify the print_position function
void print_position(const Position6axis& pos) {
  if (!g_verbose_logging) return;

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "Position: X=" << std::setw(8) << pos.x
    << " Y=" << std::setw(8) << pos.y
    << " Z=" << std::setw(8) << pos.z
    << " U=" << std::setw(8) << pos.u
    << " V=" << std::setw(8) << pos.v
    << " W=" << std::setw(8) << pos.w << std::endl;
}

// Setup controllers for our specific scenario: 3 PI and 1 ACS
// In motor_ui.cpp - Modified initialize_controllers()
void initialize_controllers() {
  // If no controllers have been created yet, set up the default ones
  if (g_controllers.empty()) {
    // Create default controllers but don't connect them
    std::cout << "No controllers defined, creating defaults..." << std::endl;

    // Create 3 PI controllers (id 0, 1, 2)
    for (int i = 0; i < 3; i++) {
      ControllerInstance pi_instance;
      pi_instance.controller = MotorControllerFactory::create_controller(MotorControllerType::PI);
      pi_instance.type = MotorControllerType::PI;
      pi_instance.id = i;
      pi_instance.name = "PI Controller " + std::to_string(i);
      g_controllers.push_back(pi_instance);
    }

    // Create 1 ACS controller (id 3)
    ControllerInstance acs_instance;
    acs_instance.controller = MotorControllerFactory::create_controller(MotorControllerType::ACS);
    acs_instance.type = MotorControllerType::ACS;
    acs_instance.id = 3;
    acs_instance.name = "ACS Controller 3";
    g_controllers.push_back(acs_instance);

    // Create Virtual controller (id 4)
    ControllerInstance virtual_instance;
    virtual_instance.controller = MotorControllerFactory::create_controller(MotorControllerType::VIRTUAL);
    virtual_instance.type = MotorControllerType::VIRTUAL;
    virtual_instance.id = 4;
    virtual_instance.name = "Virtual Controller 4";
    g_controllers.push_back(virtual_instance);
  }

  // Set the first controller as active if not already set
  if (g_controllers.size() > 0 && g_active_controller_index >= g_controllers.size()) {
    g_active_controller_index = 0;
  }

  std::cout << "Controller initialization complete. " << g_controllers.size() << " controllers registered." << std::endl;
}



// Initialize a single motor controller (legacy function maintained for compatibility)
void initialize_motor_controller(MotorControllerType type) {
  // Clear existing controllers
  g_controllers.clear();

  // Create the requested controller
  ControllerInstance instance;
  instance.controller = MotorControllerFactory::create_controller(type);
  instance.type = type;
  instance.id = 0;

  switch (type) {
  case MotorControllerType::VIRTUAL:
    instance.name = "Virtual Controller";
    break;
  case MotorControllerType::PI:
    instance.name = "PI Controller";
    break;
  case MotorControllerType::ACS:
    instance.name = "ACS Controller";
    break;
  }

  g_controllers.push_back(instance);
  g_active_controller_index = 0;
}

// Initialize a single motor controller by name (legacy function maintained for compatibility)
void initialize_motor_controller_by_name(const std::string& type_name) {
  // Clear existing controllers
  g_controllers.clear();

  // Create the requested controller
  ControllerInstance instance;
  instance.controller = MotorControllerFactory::create_controller_from_string(type_name);
  instance.id = 0;

  // Determine the actual type that was created
  std::string type = instance.controller->get_controller_type();
  if (type == "VIRTUAL") {
    instance.type = MotorControllerType::VIRTUAL;
    instance.name = "Virtual Controller";
  }
  else if (type == "PI") {
    instance.type = MotorControllerType::PI;
    instance.name = "PI Controller";
  }
  else if (type == "ACS") {
    instance.type = MotorControllerType::ACS;
    instance.name = "ACS Controller";
  }

  g_controllers.push_back(instance);
  g_active_controller_index = 0;
}

// Clean up all motor controllers
void cleanup_motor_controller() {
  for (auto& instance : g_controllers) {
    if (instance.controller && instance.controller->is_controller_connected()) {
      instance.controller->disconnect();
    }
  }
  g_controllers.clear();
}

// Get a reference to the current active motor controller
std::shared_ptr<IMotorController> get_motor_controller() {
  if (g_controllers.empty()) {
    // Initialize with a default if none exist
    initialize_motor_controller(MotorControllerType::VIRTUAL);
  }

  return g_controllers[g_active_controller_index].controller;
}

// Get the current controller type
MotorControllerType get_controller_type() {
  if (g_controllers.empty()) {
    // Initialize with a default if none exist
    initialize_motor_controller(MotorControllerType::VIRTUAL);
  }

  return g_controllers[g_active_controller_index].type;
}

// Switch between available controller instances
bool switch_controller(int index) {
  if (index >= 0 && index < g_controllers.size()) {
    g_active_controller_index = index;
    return true;
  }
  return false;
}

// Switch to a different controller type (legacy function maintained for compatibility)
bool switch_controller_type(MotorControllerType type) {
  // This function now just initializes a single controller of the requested type
  initialize_motor_controller(type);
  return true;
}

void show_motor_controller_window() {
  // Ensure controllers are initialized
  if (g_controllers.empty()) {
    initialize_controllers();
  }

  ImGui::Begin("Motor Controller Manager");

  // Create a two-column layout
  ImGui::Columns(2, "motor_controller_columns", true);
  const float column_width = ImGui::GetWindowWidth() * 0.5f;
  ImGui::SetColumnWidth(0, column_width);

  // FIRST COLUMN - Controller selection and basic info
  // Controller selection
  std::vector<const char*> controller_names;
  for (const auto& instance : g_controllers) {
    controller_names.push_back(instance.name.c_str());
  }

  ImGui::Text("Available Controllers:");
  if (ImGui::ListBox("##Controllers", &g_active_controller_index, controller_names.data(),
    controller_names.size(), 4)) {
    // User selected a different controller
    std::cout << "Switched to " << g_controllers[g_active_controller_index].name << std::endl;
  }

  ImGui::Separator();

  // Get the active controller
  auto& active_instance = g_controllers[g_active_controller_index];
  auto& controller = active_instance.controller;

  // Connection status and controls
  bool connected = controller->is_controller_connected();
  ImGui::Text("Status: %s", connected ? "Connected" : "Disconnected");
  ImGui::Text("Type: %s", controller->get_controller_type().c_str());
  ImGui::Text("ID: %d", active_instance.id);

  if (!connected) {
    static char ip_buffer[64] = "127.0.0.1";
    static int port = 8000;

    ImGui::InputText("IP Address", ip_buffer, sizeof(ip_buffer));
    ImGui::InputInt("Port", &port);

    if (ImGui::Button("Connect")) {
      if (controller->connect(ip_buffer, port)) {
        std::cout << "Motor controller connected successfully" << std::endl;
      }
      else {
        std::cerr << "Failed to connect to motor controller" << std::endl;
      }
    }
  }
  else {
    // Get current position
    Position6axis current_pos;
    if (controller->get_position(current_pos)) {
      ImGui::Text("Current Position:");
      ImGui::Text("X: %.3f", current_pos.x);
      ImGui::Text("Y: %.3f", current_pos.y);
      ImGui::Text("Z: %.3f", current_pos.z);
      ImGui::Text("U: %.3f", current_pos.u);
      ImGui::Text("V: %.3f", current_pos.v);
      ImGui::Text("W: %.3f", current_pos.w);
    }

    ImGui::Separator();
    ImGui::Text("Movement Controls:");

    // Target position input fields
    static Position6axis target_pos;

    // Use InputFloat but convert between float and double
    float x = (float)target_pos.x;
    float y = (float)target_pos.y;
    float z = (float)target_pos.z;
    float u = (float)target_pos.u;
    float v = (float)target_pos.v;
    float w = (float)target_pos.w;

    // Add - + buttons for each axis
    const float btn_width = 20.0f;
    const float input_width = column_width - 2 * btn_width - 70.0f;

    // X-axis
    ImGui::SetNextItemWidth(input_width);
    ImGui::InputFloat("##x", &x, 0.0f, 0.0f, "%.3f");
    ImGui::SameLine();
    if (ImGui::Button("-##x", ImVec2(btn_width, 0))) x -= 1.0f;
    ImGui::SameLine();
    if (ImGui::Button("+##x", ImVec2(btn_width, 0))) x += 1.0f;
    ImGui::SameLine();
    ImGui::Text("X");

    // Y-axis
    ImGui::SetNextItemWidth(input_width);
    ImGui::InputFloat("##y", &y, 0.0f, 0.0f, "%.3f");
    ImGui::SameLine();
    if (ImGui::Button("-##y", ImVec2(btn_width, 0))) y -= 1.0f;
    ImGui::SameLine();
    if (ImGui::Button("+##y", ImVec2(btn_width, 0))) y += 1.0f;
    ImGui::SameLine();
    ImGui::Text("Y");

    // Z-axis
    ImGui::SetNextItemWidth(input_width);
    ImGui::InputFloat("##z", &z, 0.0f, 0.0f, "%.3f");
    ImGui::SameLine();
    if (ImGui::Button("-##z", ImVec2(btn_width, 0))) z -= 1.0f;
    ImGui::SameLine();
    if (ImGui::Button("+##z", ImVec2(btn_width, 0))) z += 1.0f;
    ImGui::SameLine();
    ImGui::Text("Z");

    // U-axis
    ImGui::SetNextItemWidth(input_width);
    ImGui::InputFloat("##u", &u, 0.0f, 0.0f, "%.3f");
    ImGui::SameLine();
    if (ImGui::Button("-##u", ImVec2(btn_width, 0))) u -= 1.0f;
    ImGui::SameLine();
    if (ImGui::Button("+##u", ImVec2(btn_width, 0))) u += 1.0f;
    ImGui::SameLine();
    ImGui::Text("U");

    // V-axis
    ImGui::SetNextItemWidth(input_width);
    ImGui::InputFloat("##v", &v, 0.0f, 0.0f, "%.3f");
    ImGui::SameLine();
    if (ImGui::Button("-##v", ImVec2(btn_width, 0))) v -= 1.0f;
    ImGui::SameLine();
    if (ImGui::Button("+##v", ImVec2(btn_width, 0))) v += 1.0f;
    ImGui::SameLine();
    ImGui::Text("V");

    // W-axis
    ImGui::SetNextItemWidth(input_width);
    ImGui::InputFloat("##w", &w, 0.0f, 0.0f, "%.3f");
    ImGui::SameLine();
    if (ImGui::Button("-##w", ImVec2(btn_width, 0))) w -= 1.0f;
    ImGui::SameLine();
    if (ImGui::Button("+##w", ImVec2(btn_width, 0))) w += 1.0f;
    ImGui::SameLine();
    ImGui::Text("W");

    // Update the target position with converted values
    target_pos.x = (double)x;
    target_pos.y = (double)y;
    target_pos.z = (double)z;
    target_pos.u = (double)u;
    target_pos.v = (double)v;
    target_pos.w = (double)w;

    // Movement controls
    ImGui::Text("Status: %s", controller->is_controller_moving() ? "Moving" : "Idle");

    if (ImGui::Button("Move to Target")) {
      if (controller->move_to_point(target_pos)) {
        std::cout << "Moving to target position..." << std::endl;
      }
    }

    ImGui::SameLine();

    if (ImGui::Button("Home Position")) {
      Position6axis home_pos;  // Defaults to all zeros
      controller->move_to_point(home_pos);
    }

    // ACS-specific emergency stop button
    if (active_instance.type == MotorControllerType::ACS) {
      ImGui::SameLine();
      if (ImGui::Button("EMERGENCY STOP")) {
        // Dynamic cast to access ACS-specific methods
        auto acs_controller = std::dynamic_pointer_cast<ACSMotorController>(controller);
        if (acs_controller) {
          acs_controller->emergency_stop();
        }
      }
    }

    // Switch to second column
    ImGui::NextColumn();

    // SECOND COLUMN - Movement params, saved positions, JSON status

    // Movement parameters
    ImGui::Text("Movement Parameters:");

    static double speed = 10.0;
    static double accel = 5.0;
    static double tolerance = 0.001;

    bool params_changed = false;

    // Use SliderFloat but convert to/from double
    float speed_float = (float)speed;
    float accel_float = (float)accel;
    float tolerance_float = (float)tolerance;

    params_changed |= ImGui::SliderFloat("Speed", &speed_float, 1.0f, 100.0f, "%.1f");
    params_changed |= ImGui::SliderFloat("Acceleration", &accel_float, 1.0f, 50.0f, "%.1f");

    // Position tolerance with +/- buttons
    ImGui::SetNextItemWidth(ImGui::GetColumnWidth() - 50.0f);
    params_changed |= ImGui::InputFloat("Position Tolerance", &tolerance_float, 0.0001f, 0.01f, "%.6f");
    ImGui::SameLine();
    if (ImGui::Button("-##tol", ImVec2(btn_width, 0))) tolerance_float = std::max(tolerance_float - 0.0001f, 0.0001f);
    ImGui::SameLine();
    if (ImGui::Button("+##tol", ImVec2(btn_width, 0))) tolerance_float += 0.0001f;

    // Convert back to double
    if (params_changed) {
      speed = (double)speed_float;
      accel = (double)accel_float;
      tolerance = (double)tolerance_float;
      controller->set_movement_parameters(speed, accel, tolerance);
    }

    // Saved positions
    ImGui::Separator();
    if (ImGui::CollapsingHeader("Saved Positions", ImGuiTreeNodeFlags_DefaultOpen)) {
      static char pos_name[64] = "";
      // Using a map of maps to store positions per controller
      static std::map<int, std::map<std::string, Position6axis>> saved_positions_map;

      ImGui::InputText("Position Name", pos_name, sizeof(pos_name));

      if (ImGui::Button("Save Current")) {
        if (strlen(pos_name) > 0) {
          Position6axis current;
          if (controller->get_position(current)) {
            saved_positions_map[active_instance.id][pos_name] = current;
            pos_name[0] = '\0'; // Clear the input
          }
        }
      }

      ImGui::Separator();

      auto& saved_positions = saved_positions_map[active_instance.id];
      if (!saved_positions.empty()) {
        ImGui::Text("Saved Positions for %s:", active_instance.name.c_str());

        std::vector<std::string> positions_to_remove;

        for (const auto& [name, pos] : saved_positions) {
          ImGui::PushID(name.c_str());

          if (ImGui::Button("Go")) {
            controller->move_to_point(pos);
          }

          ImGui::SameLine();

          if (ImGui::Button("X")) {
            positions_to_remove.push_back(name);
          }

          ImGui::SameLine();
          ImGui::Text("%s (%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)",
            name.c_str(), pos.x, pos.y, pos.z, pos.u, pos.v, pos.w);

          ImGui::PopID();
        }

        // Remove positions that were marked for deletion
        for (const auto& name : positions_to_remove) {
          saved_positions.erase(name);
        }
      }
    }

    // Status JSON display
    ImGui::Separator();
    if (ImGui::CollapsingHeader("Controller JSON Status")) {
      static char status_json[4096] = "";
      if (ImGui::Button("Refresh JSON Status")) {
        std::string json = controller->get_status_json();
        strncpy(status_json, json.c_str(), sizeof(status_json) - 1);
        status_json[sizeof(status_json) - 1] = '\0';
      }

      ImGui::InputTextMultiline("##status_json", status_json, sizeof(status_json),
        ImVec2(-1.0f, ImGui::GetTextLineHeight() * 10),
        ImGuiInputTextFlags_ReadOnly);
    }

    // Return to first column for disconnect button
    ImGui::NextColumn();

    // Disconnect button
    ImGui::Separator();
    if (ImGui::Button("Disconnect")) {
      controller->disconnect();
      std::cout << "Motor controller disconnected" << std::endl;
    }
  }

  // Reset column layout
  ImGui::Columns(1);
  ImGui::End();
}

// Connect all controllers (utility function)
void connect_all_controllers(const std::string& ip_base, int start_port) {
  for (size_t i = 0; i < g_controllers.size(); i++) {
    auto& instance = g_controllers[i];

    // If already connected, skip
    if (instance.controller->is_controller_connected()) {
      continue;
    }

    // Generate IP with last octet based on controller ID
    std::string ip = ip_base + std::to_string(100 + instance.id);
    int port = start_port + instance.id;

    std::cout << "Connecting " << instance.name << " to " << ip << ":" << port << std::endl;

    if (instance.controller->connect(ip, port)) {
      std::cout << "Connected successfully!" << std::endl;
    }
    else {
      std::cerr << "Failed to connect " << instance.name << std::endl;
    }
  }
}

// Disconnect all controllers (utility function)
void disconnect_all_controllers() {
  for (auto& instance : g_controllers) {
    if (instance.controller->is_controller_connected()) {
      instance.controller->disconnect();
      std::cout << "Disconnected " << instance.name << std::endl;
    }
  }
}

// Add this to motor_ui.cpp
bool connect_controller(int id, const std::string& ip, int port) {
  // Find the controller with the given ID
  for (auto& instance : g_controllers) {
    if (instance.id == id) {
      if (instance.controller->is_controller_connected()) {
        std::cout << instance.name << " is already connected" << std::endl;
        return true;
      }

      std::cout << "Connecting " << instance.name << " to " << ip << ":" << port << std::endl;
      if (instance.controller->connect(ip, port)) {
        std::cout << "Connected successfully!" << std::endl;
        return true;
      }
      else {
        std::cerr << "Failed to connect " << instance.name << std::endl;
        return false;
      }
    }
  }

  std::cerr << "Controller with ID " << id << " not found" << std::endl;
  return false;
}


// Add this to motor_ui.cpp
bool connect_controller(int id, const std::string& ip, int port, MotorControllerType type) {
  // Check if a controller with this ID already exists
  for (auto& instance : g_controllers) {
    if (instance.id == id) {
      // Controller exists, try to connect it
      if (instance.controller->is_controller_connected()) {
        std::cout << instance.name << " is already connected" << std::endl;
        return true;
      }

      std::cout << "Connecting " << instance.name << " to " << ip << ":" << port << std::endl;
      if (instance.controller->connect(ip, port)) {
        std::cout << "Connected successfully!" << std::endl;
        return true;
      }
      else {
        std::cerr << "Failed to connect " << instance.name << std::endl;
        return false;
      }
    }
  }

  // Controller doesn't exist, create it
  ControllerInstance new_instance;
  new_instance.controller = MotorControllerFactory::create_controller(type);
  new_instance.type = type;
  new_instance.id = id;

  // Set name based on type
  switch (type) {
  case MotorControllerType::PI:
    new_instance.name = "PI Controller " + std::to_string(id);
    break;
  case MotorControllerType::ACS:
    new_instance.name = "ACS Controller " + std::to_string(id);
    break;
  case MotorControllerType::VIRTUAL:
    new_instance.name = "Virtual Controller " + std::to_string(id);
    break;
  default:
    new_instance.name = "Unknown Controller " + std::to_string(id);
  }

  // Add to the controllers list
  g_controllers.push_back(new_instance);

  // Now connect it
  std::cout << "Created and connecting " << new_instance.name << " to " << ip << ":" << port << std::endl;
  if (new_instance.controller->connect(ip, port)) {
    std::cout << "Connected successfully!" << std::endl;
    return true;
  }
  else {
    std::cerr << "Failed to connect " << new_instance.name << std::endl;
    return false;
  }
}

// Add to motor_ui.cpp
void set_verbose_logging(bool verbose) {
  g_verbose_logging = verbose;
}