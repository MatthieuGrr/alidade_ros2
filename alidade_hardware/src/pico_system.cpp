#include "alidade_hardware/pico_system.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace alidade_hardware
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace
{

constexpr double kPi = 3.14159265358979323846;

double param_or(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  double fallback, bool required = false)
{
  auto it = params.find(key);
  if (it == params.end()) {
    if (required) {
      throw std::runtime_error("parametre <hardware> manquant : " + key);
    }
    return fallback;
  }
  return std::stod(it->second);
}

}  // namespace

CallbackReturn PicoSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    const auto & p = info_.hardware_parameters;
    auto it = p.find("port");
    port_ = it != p.end() ? it->second : "/dev/pico";
    counts_per_rev_ = param_or(p, "counts_per_rev", 0.0, true);
    wheel_max_rad_s_ = param_or(p, "wheel_max_rad_s", 0.0, true);
    frame_rate_hz_ = param_or(p, "frame_rate_hz", 50.0);
    watchdog_ms_ = param_or(p, "watchdog_ms", 300.0);
    accel_lsb_per_m_s2_ = param_or(p, "accel_lsb_per_m_s2", 100.0);
    gyro_lsb_per_deg_s_ = param_or(p, "gyro_lsb_per_deg_s", 16.0);
    // Tolere quelques trames perdues, mais reste bien en deca de ce qu'un
    // navigateur accepterait comme odometrie figee.
    link_timeout_s_ = param_or(p, "link_timeout_s", 0.5);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }
  if (counts_per_rev_ <= 0.0 || wheel_max_rad_s_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "counts_per_rev et wheel_max_rad_s doivent etre positifs");
    return CallbackReturn::ERROR;
  }
  rad_per_tick_ = 2.0 * kPi / counts_per_rev_;

  // Deux roues, dans l'ordre gauche puis droite du bloc ros2_control, avec
  // exactement les interfaces que diff_drive_controller attend.
  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(get_logger(), "deux joints attendus, %zu trouves", info_.joints.size());
    return CallbackReturn::ERROR;
  }
  for (const auto & joint : info_.joints) {
    const bool command_ok = joint.command_interfaces.size() == 1 &&
      joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY;
    bool has_position = false;
    bool has_velocity = false;
    for (const auto & s : joint.state_interfaces) {
      has_position |= s.name == hardware_interface::HW_IF_POSITION;
      has_velocity |= s.name == hardware_interface::HW_IF_VELOCITY;
    }
    if (!command_ok || !has_position || !has_velocity) {
      RCLCPP_ERROR(
        get_logger(), "joint %s : commande velocity et etats position+velocity requis",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    joint_names_.push_back(joint.name);
  }
  if (!info_.sensors.empty()) {
    imu_name_ = info_.sensors[0].name;
  }

  RCLCPP_INFO(
    get_logger(), "Pico sur %s : %.0f counts/tour (%.4e rad/tick), roue max %.2f rad/s, "
    "trames a %.0f Hz, chien de garde %.0f ms",
    port_.c_str(), counts_per_rev_, rad_per_tick_, wheel_max_rad_s_, frame_rate_hz_, watchdog_ms_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn PicoSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    serial_.open(port_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return CallbackReturn::FAILURE;
  }
  {
    std::lock_guard<std::mutex> lock(link_mutex_);
    link_ = Link{};
  }
  reader_running_ = true;
  reader_ = std::thread(&PicoSystem::reader_loop, this);

  if (auto node = get_node()) {
    diagnostics_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));
  } else {
    RCLCPP_WARN(get_logger(), "pas de noeud disponible : diagnostics desactives");
  }
  RCLCPP_INFO(get_logger(), "port %s ouvert, lecteur demarre", port_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn PicoSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop_reader();
  std::lock_guard<std::mutex> lock(serial_mutex_);
  serial_.close();
  diagnostics_pub_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn PicoSystem::on_activate(const rclcpp_lifecycle::State &)
{
  for (const auto & name : joint_names_) {
    set_command(name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
  }
  last_cmd_[0] = last_cmd_[1] = 0;
  if (!send(encode_velocity(0, 0))) {
    return CallbackReturn::FAILURE;
  }

  // Un Pico muet ne doit pas passer pour actif : on exige une trame fraiche
  // avant de rendre la main aux controleurs.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(link_mutex_);
      if (link_.have_frame) {
        RCLCPP_INFO(
          get_logger(), "Pico actif : t=%u ms, enc=(%d, %d)", link_.frame.t_ms, link_.frame.enc[0],
          link_.frame.enc[1]);
        return CallbackReturn::SUCCESS;
      }
      if (!link_.failure.empty()) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  RCLCPP_ERROR(get_logger(), "aucune trame S recue du Pico sur %s", port_.c_str());
  return CallbackReturn::FAILURE;
}

CallbackReturn PicoSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  send(encode_stop());
  return CallbackReturn::SUCCESS;
}

CallbackReturn PicoSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  send(encode_stop());
  stop_reader();
  std::lock_guard<std::mutex> lock(serial_mutex_);
  serial_.close();
  return CallbackReturn::SUCCESS;
}

CallbackReturn PicoSystem::on_error(const rclcpp_lifecycle::State &)
{
  // Le firmware coupe de lui-meme les moteurs 300 ms apres la derniere
  // commande : on libere simplement le port. Le composant repasse en
  // 'unconfigured' ; le relancer (ou relancer le service) le reconnecte.
  stop_reader();
  std::lock_guard<std::mutex> lock(serial_mutex_);
  serial_.close();
  RCLCPP_ERROR(get_logger(), "liaison Pico en erreur, port %s libere", port_.c_str());
  return CallbackReturn::SUCCESS;
}

void PicoSystem::reader_loop()
{
  std::string pending;
  char buffer[512];
  while (reader_running_) {
    size_t n = 0;
    try {
      n = serial_.read_some(buffer, sizeof(buffer), std::chrono::milliseconds(50));
    } catch (const std::exception & e) {
      std::lock_guard<std::mutex> lock(link_mutex_);
      link_.failure = e.what();
      return;
    }
    if (n == 0) {
      continue;
    }
    pending.append(buffer, n);

    size_t newline;
    while ((newline = pending.find('\n')) != std::string::npos) {
      std::string line = pending.substr(0, newline);
      pending.erase(0, newline + 1);
      if (line.empty()) {
        continue;
      }
      ParseError why;
      auto frame = parse_state_frame(line, &why);
      std::lock_guard<std::mutex> lock(link_mutex_);
      if (frame) {
        link_.frame = *frame;
        link_.received_at = std::chrono::steady_clock::now();
        link_.have_frame = true;
        ++link_.frames_received;
      } else {
        ++link_.frames_rejected;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "trame rejetee (%s) : %.60s", to_string(why),
          line.c_str());
      }
    }
    // Une ligne sans fin depuis longtemps est du bruit : on la jette plutot
    // que de laisser le tampon grossir.
    if (pending.size() > 1024) {
      std::lock_guard<std::mutex> lock(link_mutex_);
      link_.bytes_dropped += pending.size();
      pending.clear();
    }
  }
}

void PicoSystem::stop_reader()
{
  reader_running_ = false;
  if (reader_.joinable()) {
    reader_.join();
  }
}

bool PicoSystem::send(const std::string & command)
{
  try {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    serial_.write_all(command);
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "envoi impossible : %s", e.what());
    return false;
  }
}

return_type PicoSystem::read(const rclcpp::Time & time, const rclcpp::Duration &)
{
  Link snapshot;
  {
    std::lock_guard<std::mutex> lock(link_mutex_);
    snapshot = link_;
  }

  if (!snapshot.failure.empty()) {
    RCLCPP_ERROR(get_logger(), "liaison Pico perdue : %s", snapshot.failure.c_str());
    return return_type::ERROR;
  }

  const double age_s = snapshot.have_frame
    ? std::chrono::duration<double>(std::chrono::steady_clock::now() - snapshot.received_at).count()
    : 0.0;
  if (snapshot.have_frame && age_s > link_timeout_s_) {
    RCLCPP_ERROR(
      get_logger(), "aucune trame S depuis %.0f ms (limite %.0f ms)", age_s * 1e3,
      link_timeout_s_ * 1e3);
    publish_diagnostics(time, age_s, false);
    return return_type::ERROR;
  }

  if (snapshot.have_frame) {
    const auto & f = snapshot.frame;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      set_state(joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION, f.enc[i] * rad_per_tick_);
      set_state(joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY, f.vel[i] * rad_per_tick_);
    }
    if (!imu_name_.empty()) {
      // Pas d'orientation dans la trame : quaternion identite, et le
      // broadcaster publie une covariance a -1 pour le signaler.
      set_state(imu_name_ + "/orientation.x", 0.0);
      set_state(imu_name_ + "/orientation.y", 0.0);
      set_state(imu_name_ + "/orientation.z", 0.0);
      set_state(imu_name_ + "/orientation.w", 1.0);
      const double gyro_scale = (kPi / 180.0) / gyro_lsb_per_deg_s_;
      const double accel_scale = 1.0 / accel_lsb_per_m_s2_;
      const char * axes[3] = {"x", "y", "z"};
      for (int i = 0; i < 3; ++i) {
        set_state(imu_name_ + "/angular_velocity." + axes[i], f.gyro[i] * gyro_scale);
        set_state(imu_name_ + "/linear_acceleration." + axes[i], f.accel[i] * accel_scale);
      }
    }
  }

  publish_diagnostics(time, age_s, true);
  return return_type::OK;
}

return_type PicoSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  int32_t ticks[2];
  for (size_t i = 0; i < 2; ++i) {
    double rad_s = get_command(joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY);
    if (!std::isfinite(rad_s)) {
      rad_s = 0.0;
    }
    // Le firmware n'applique aucune limite : c'est ici qu'on borne.
    rad_s = std::clamp(rad_s, -wheel_max_rad_s_, wheel_max_rad_s_);
    ticks[i] = static_cast<int32_t>(std::lround(rad_s / rad_per_tick_));
  }
  last_cmd_[0] = ticks[0];
  last_cmd_[1] = ticks[1];
  // Envoye a chaque cycle, meme nul : c'est ce qui rearme le chien de garde.
  return send(encode_velocity(ticks[0], ticks[1])) ? return_type::OK : return_type::ERROR;
}

void PicoSystem::publish_diagnostics(const rclcpp::Time & time, double frame_age_s, bool link_ok)
{
  if (!diagnostics_pub_) {
    return;
  }
  if (last_diagnostics_.nanoseconds() != 0 && (time - last_diagnostics_).seconds() < 1.0 && link_ok) {
    return;
  }
  last_diagnostics_ = time;

  Link snapshot;
  {
    std::lock_guard<std::mutex> lock(link_mutex_);
    snapshot = link_;
  }

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "alidade_hardware: Pico";
  status.hardware_id = port_;
  const double rejected_ratio = snapshot.frames_received + snapshot.frames_rejected > 0
    ? static_cast<double>(snapshot.frames_rejected) /
      static_cast<double>(snapshot.frames_received + snapshot.frames_rejected)
    : 0.0;
  if (!link_ok) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "liaison perdue";
  } else if (rejected_ratio > 0.01) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "trames rejetees";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "ok";
  }
  auto add = [&](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  };
  add("trames recues", std::to_string(snapshot.frames_received));
  add("trames rejetees", std::to_string(snapshot.frames_rejected));
  add("octets jetes", std::to_string(snapshot.bytes_dropped));
  add("age derniere trame (ms)", std::to_string(static_cast<int>(frame_age_s * 1e3)));
  add("t_ms Pico", std::to_string(snapshot.frame.t_ms));
  add("consigne (ticks/s)", std::to_string(last_cmd_[0]) + " " + std::to_string(last_cmd_[1]));
  add("imu presente", (snapshot.frame.accel[0] | snapshot.frame.accel[1] | snapshot.frame.accel[2]) ? "oui" : "non");

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = time;
  array.status.push_back(status);
  diagnostics_pub_->publish(array);
}

}  // namespace alidade_hardware

PLUGINLIB_EXPORT_CLASS(alidade_hardware::PicoSystem, hardware_interface::SystemInterface)
