// rssi_sim.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/home_position.hpp>
#include <mavros_msgs/msg/radio_status.hpp>

#include <algorithm>
#include <random>
#include <vector>
#include <string>
#include <cmath>
#include <mutex>
#include <optional>
#include <array>

using std::placeholders::_1;

class RssiSimNode : public rclcpp::Node {
public:
  RssiSimNode() : Node("rssi_sim")
  {
    // --- Parameters (physics: unchanged, just exposed) ---
    mode_         = this->declare_parameter<std::string>("mode", "ideal"); // "ideal" or "noisy"
    P0_           = this->declare_parameter<double>("P0",   -40.0);
    n_            = this->declare_parameter<double>("n",     1.465);
    d0_           = this->declare_parameter<double>("d0",    1.0);
    // Defaults from Wojcicki et al. (Sensors 2021) – Real scenario Table 2
    dist_keys_    = this->declare_parameter<std::vector<double>>("dist_keys",
                      std::vector<double>{1,5,10,15,20,25,35,50,75,100});
    sigma_vals_   = this->declare_parameter<std::vector<double>>("sigma_vals",
                      std::vector<double>{6.0211,6.0380,4.4740,4.1058,4.8169,
                                          4.2999,5.7238,5.9673,5.0651,4.2553});
    rng_seed_     = this->declare_parameter<int>("rng_seed", 42);
    pub_rate_hz_  = this->declare_parameter<double>("publish_rate_hz", 10.0);

    // --- Mapping-related tunables (do not affect propagation model) ---
    noise_dbm_        = this->declare_parameter<double>("noise_dbm", -45.0);         // typical SiK reported noise
    rem_asym_sigma_db_= this->declare_parameter<double>("rem_asym_sigma_db", 1.0);   // remote asymmetry (stdev, dB)

    // Validate arrays
    if (dist_keys_.size() != sigma_vals_.size() || dist_keys_.empty()) {
      RCLCPP_WARN(get_logger(), "dist_keys and sigma_vals size mismatch or empty; falling back to single sigma=4.0");
      dist_keys_  = {1.0, 100.0};
      sigma_vals_ = {4.0, 4.0};
    }
    // Ensure strictly increasing dist_keys_ (keep sigma pairs aligned)
    for (size_t i=1; i<dist_keys_.size(); ++i) {
      if (dist_keys_[i] <= dist_keys_[i-1]) {
        RCLCPP_WARN(get_logger(), "dist_keys must be strictly increasing; fixing order by sorting paired arrays");
        std::vector<std::pair<double,double>> pairs;
        pairs.reserve(dist_keys_.size());
        for (size_t k=0;k<dist_keys_.size();++k) pairs.emplace_back(dist_keys_[k], sigma_vals_[k]);
        std::sort(pairs.begin(), pairs.end(), [](auto &a, auto &b){return a.first<b.first;});
        for (size_t k=0;k<pairs.size();++k){ dist_keys_[k]=pairs[k].first; sigma_vals_[k]=pairs[k].second; }
        break;
      }
    }

    // RNG
    rng_ = std::mt19937(rng_seed_);

    // Subscribers
    sub_home_ = this->create_subscription<mavros_msgs::msg::HomePosition>(
      "/mavros/home_position/home", rclcpp::QoS(1).transient_local(),
      std::bind(&RssiSimNode::homeCb, this, _1));

    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      std::bind(&RssiSimNode::poseCb, this, _1));

    // Publisher
    pub_radio_ = this->create_publisher<mavros_msgs::msg::RadioStatus>("/mavros/radio_status", 10);

    // Timer
    using namespace std::chrono;
    auto period = duration<double>(1.0 / std::max(1.0, pub_rate_hz_));
    timer_ = this->create_wall_timer(duration_cast<milliseconds>(period),
             std::bind(&RssiSimNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "rssi_sim started (mode=%s, P0=%.2f dBm, n=%.3f, d0=%.2f m, rate=%.1f Hz, noise_dbm=%.1f, rem_sigma=%.1f dB)",
      mode_.c_str(), P0_, n_, d0_, pub_rate_hz_, noise_dbm_, rem_asym_sigma_db_);
  }

private:
  // --- Callbacks ---
  void homeCb(const mavros_msgs::msg::HomePosition::SharedPtr msg) {
    std::scoped_lock lk(mtx_);
    home_xyz_ = { msg->position.x, msg->position.y, msg->position.z };
  }

  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::scoped_lock lk(mtx_);
    current_xyz_ = { msg->pose.position.x, msg->pose.position.y, msg->pose.position.z };
  }

  // --- Timer: compute & publish ---
  void onTimer() {
    std::optional<std::array<double,3>> home, cur;
    {
      std::scoped_lock lk(mtx_);
      home = home_xyz_;
      cur  = current_xyz_;
    }
    if (!home || !cur) {
      // Need both to compute distance
      return;
    }

    const double dx = (*cur)[0] - (*home)[0];
    const double dy = (*cur)[1] - (*home)[1];
    const double dz = (*cur)[2] - (*home)[2];
    double d  = std::sqrt(dx*dx + dy*dy + dz*dz);
    d = std::max(d, d0_); // clamp to reference distance

    // --- Propagation model (unchanged) ---
    double rssi_dbm = rssiIdeal(d);
    if (mode_ == "noisy") {
      const double sigma = sigmaOf(d);
      std::normal_distribution<double> gauss(0.0, sigma);
      rssi_dbm += gauss(rng_);
    }

    // --- Mapping: dBm <-> raw SiK units (0..254, 255==unknown) ---
    const uint8_t rssi_raw = dBmToRawRSSI(rssi_dbm);

    // Remote asymmetry (small, in dB)
    double rem_dbm = rssi_dbm;
    if (rem_asym_sigma_db_ > 0.0) {
      std::normal_distribution<double> small_dbm(0.0, (mode_=="noisy") ? rem_asym_sigma_db_ : 0.0);
      rem_dbm += small_dbm(rng_);
    }
    const uint8_t rem_raw = dBmToRawRSSI(rem_dbm);

    // Background noise reported by SiK (use constant or make dynamic if you like)
    const uint8_t noise_raw = dBmNoiseToRaw(noise_dbm_);

    mavros_msgs::msg::RadioStatus msg;
    // raw device units (0..254)
    msg.rssi     = rssi_raw;
    msg.remrssi  = rem_raw;
    msg.noise    = noise_raw;
    msg.remnoise = noise_raw;

    // txbuf is "remaining TX buffer" (0..100). Keep simple or simulate congestion.
    msg.txbuf    = 100;

    // error counters and fixed flag (leave simple; you can increment to simulate loss)
    msg.rxerrors = 0;
    msg.fixed    = 0;

    // dBm (floats) for consumers that display/record calibrated values
    msg.rssi_dbm    = static_cast<float>(rssi_dbm);
    msg.remrssi_dbm = static_cast<float>(rem_dbm);

    pub_radio_->publish(msg);
  }

  // --- Model pieces (unchanged physics) ---
  double rssiIdeal(double distance_m) const {
    // RSSI(dBm) = P0 - 10 n log10(d / d0)
    return P0_ - 10.0 * n_ * std::log10(distance_m / d0_);
  }

  double sigmaOf(double distance_m) const {
    // linear interpolation over (dist_keys_, sigma_vals_)
    if (distance_m <= dist_keys_.front()) return sigma_vals_.front();
    if (distance_m >= dist_keys_.back())  return sigma_vals_.back();

    auto it = std::upper_bound(dist_keys_.begin(), dist_keys_.end(), distance_m);
    size_t idx = static_cast<size_t>(std::distance(dist_keys_.begin(), it));
    // segment [idx-1, idx]
    const double x0 = dist_keys_[idx-1], x1 = dist_keys_[idx];
    const double y0 = sigma_vals_[idx-1], y1 = sigma_vals_[idx];
    const double t = (distance_m - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
  }

  // --- SiK/ArduPilot mapping helpers ---
  static uint8_t clampU8_254(double v) {
    if (v < 0.0)   return 0;
    if (v > 254.0) return 254; // 255 == unknown/invalid
    return static_cast<uint8_t>(std::lround(v));
  }
  static uint8_t dBmToRawRSSI(double dbm) {
    // ArduPilot/SiK approx: RSSI_raw ≈ 1.9 * (dBm + 127)
    return clampU8_254(1.9 * (dbm + 127.0));
  }
  static double rawRSSIToDbm(uint8_t raw) {
    // Inverse: dBm ≈ (raw / 1.9) - 127
    return (static_cast<double>(raw) / 1.9) - 127.0;
  }
  static uint8_t dBmNoiseToRaw(double noise_dbm) {
    // Same mapping for noise
    return dBmToRawRSSI(noise_dbm);
  }

  // --- State ---
  std::mutex mtx_;
  std::optional<std::array<double,3>> home_xyz_;
  std::optional<std::array<double,3>> current_xyz_;

  // Parameters
  std::string mode_;
  double P0_, n_, d0_;
  std::vector<double> dist_keys_;
  std::vector<double> sigma_vals_;
  int rng_seed_;
  double pub_rate_hz_;
  double noise_dbm_;
  double rem_asym_sigma_db_;

  // RNG
  std::mt19937 rng_;

  // ROS
  rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr sub_home_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Publisher<mavros_msgs::msg::RadioStatus>::SharedPtr pub_radio_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RssiSimNode>());
  rclcpp::shutdown();
  return 0;
}
