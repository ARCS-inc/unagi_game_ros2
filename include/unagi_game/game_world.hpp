#pragma once

#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace unagi_game
{

struct ObstacleData
{
  double x{0.0};
  double y{0.0};
  double radius{0.0};
};

struct PhysicsConfig
{
  double world_length{55.0};
  double arena_half_height{4.0};
  double unagi_radius{0.24};
  double mass{1.0};
  double inertia{0.20};
  double thrust{8.0};
  double lateral_thrust{3.0};
  double steering_torque{4.5};
  double linear_drag{1.8};
  double angular_drag{3.8};
  double max_speed{5.2};
  double max_omega{4.5};
  double start_speed{1.5};

  int obstacle_count{18};
  double obstacle_radius_min{0.45};
  double obstacle_radius_max{0.85};
  bool randomize_obstacles_each_round{true};
  int64_t obstacle_seed{-1};
  double obstacle_start_x{6.0};
  double obstacle_end_margin{4.0};
  double obstacle_x_jitter{0.30};
  double obstacle_wall_margin{0.15};
  double obstacle_path_margin{0.55};
  double path_max_shift{0.85};
};

class GameWorld
{
public:
  explicit GameWorld(const PhysicsConfig & config = PhysicsConfig{});

  void regenerate_obstacles();
  void reset(std::optional<bool> regenerate_obstacles = std::nullopt);
  void set_flap(double x, double y);
  void step(double dt, double time_limit);
  int score() const;

  const PhysicsConfig & config() const {return cfg_;}
  const std::vector<ObstacleData> & obstacles() const {return obstacles_;}
  const std::vector<std::pair<double, double>> & safe_path() const {return safe_path_;}

  uint32_t round_index() const {return round_index_;}
  int64_t layout_seed() const {return layout_seed_;}
  double unagi_x() const {return unagi_x_;}
  double unagi_y() const {return unagi_y_;}
  double unagi_theta() const {return unagi_theta_;}
  double unagi_vx() const {return unagi_vx_;}
  double unagi_vy() const {return unagi_vy_;}
  double unagi_omega() const {return unagi_omega_;}
  double elapsed() const {return elapsed_;}
  double flap_x() const {return flap_x_;}
  double flap_y() const {return flap_y_;}
  bool finished() const {return finished_;}
  bool success() const {return success_;}
  const std::string & reason() const {return reason_;}

  void finish(bool success, const std::string & reason);

private:
  static std::pair<double, double> normalize_flap(double x, double y);
  std::optional<std::string> collision_reason() const;
  std::pair<std::vector<ObstacleData>, std::vector<std::pair<double, double>>>
  generate_obstacles(int64_t layout_seed) const;
  int64_t next_layout_seed();

  PhysicsConfig cfg_;
  std::mt19937 seed_source_;
  bool nondeterministic_seed_{false};

  uint32_t round_index_{0};
  int64_t layout_seed_{0};
  std::vector<ObstacleData> obstacles_;
  std::vector<std::pair<double, double>> safe_path_;

  double unagi_x_{1.0};
  double unagi_y_{0.0};
  double unagi_theta_{0.0};
  double unagi_vx_{0.0};
  double unagi_vy_{0.0};
  double unagi_omega_{0.0};
  double elapsed_{0.0};
  double flap_x_{1.0};
  double flap_y_{0.0};
  bool finished_{false};
  bool success_{false};
  std::string reason_;
};

}  // namespace unagi_game
