// Interface ros2_control vers diffdrive_pico_firmware.
//
// Le Pico ne connait que des ticks : toute la conversion vers les radians et
// les unites SI est faite ici, a partir des parametres du bloc <hardware> de
// l'URDF, eux-memes issus de la revision materielle.
//
// Un thread lecteur consomme les trames S a 50 Hz et garde la derniere ;
// read() la copie dans les interfaces d'etat, write() envoie la consigne
// a chaque cycle, y compris nulle, pour tenir le chien de garde du firmware.
#ifndef ALIDADE_HARDWARE__PICO_SYSTEM_HPP_
#define ALIDADE_HARDWARE__PICO_SYSTEM_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "alidade_hardware/pico_protocol.hpp"
#include "alidade_hardware/serial_port.hpp"

namespace alidade_hardware
{

class PicoSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PicoSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Link
  {
    StateFrame frame;
    std::chrono::steady_clock::time_point received_at{};
    bool have_frame = false;
    uint64_t frames_received = 0;
    uint64_t frames_rejected = 0;
    uint64_t bytes_dropped = 0;
    std::string failure;  // vide tant que le lien est sain
  };

  void reader_loop();
  void stop_reader();
  bool send(const std::string & command);
  void publish_diagnostics(const rclcpp::Time & time, double frame_age_s, bool link_ok);

  // Parametres, issus de <hardware><param>.
  std::string port_;
  double counts_per_rev_ = 0.0;
  double wheel_max_rad_s_ = 0.0;
  double frame_rate_hz_ = 50.0;
  double watchdog_ms_ = 300.0;
  double accel_lsb_per_m_s2_ = 100.0;
  double gyro_lsb_per_deg_s_ = 16.0;
  double link_timeout_s_ = 0.5;

  double rad_per_tick_ = 0.0;
  std::vector<std::string> joint_names_;
  std::string imu_name_;

  SerialPort serial_;
  std::mutex serial_mutex_;
  std::thread reader_;
  std::atomic<bool> reader_running_{false};

  std::mutex link_mutex_;
  Link link_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Time last_diagnostics_{0, 0, RCL_ROS_TIME};
  int32_t last_cmd_[2] = {0, 0};
  // Origine des compteurs, prise a l'activation : le Pico compte depuis SON
  // demarrage, pas depuis le notre (voir on_activate).
  int32_t enc_origin_[2] = {0, 0};
};

}  // namespace alidade_hardware

#endif  // ALIDADE_HARDWARE__PICO_SYSTEM_HPP_
