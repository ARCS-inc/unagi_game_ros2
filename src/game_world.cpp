#include "unagi_game/game_world.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace unagi_game
{
namespace
{

double uniform(std::mt19937 & rng, double lo, double hi)
{
  if (hi <= lo) {
    return lo;
  }
  return std::uniform_real_distribution<double>(lo, hi)(rng);
}

}  // namespace

GameWorld::GameWorld(const PhysicsConfig & config)
: cfg_(config)
{
  if (cfg_.obstacle_seed >= 0) {
    seed_source_.seed(static_cast<std::mt19937::result_type>(cfg_.obstacle_seed));
  } else {
    std::random_device rd;
    std::seed_seq seq{rd(), rd(), rd(), rd()};
    seed_source_.seed(seq);
    nondeterministic_seed_ = true;
  }
  regenerate_obstacles();
  reset(false);
}

int64_t GameWorld::next_layout_seed()
{
  (void)nondeterministic_seed_;
  std::uniform_int_distribution<int64_t> dist(0, 2147483646LL);
  return dist(seed_source_);
}

void GameWorld::regenerate_obstacles()
{
  ++round_index_;
  layout_seed_ = next_layout_seed();
  auto generated = generate_obstacles(layout_seed_);
  obstacles_ = std::move(generated.first);
  safe_path_ = std::move(generated.second);
}

void GameWorld::reset(std::optional<bool> regenerate)
{
  const bool should_regenerate = regenerate.value_or(cfg_.randomize_obstacles_each_round);
  if (should_regenerate) {
    regenerate_obstacles();
  }

  unagi_x_ = 1.0;
  unagi_y_ = 0.0;
  unagi_theta_ = 0.0;
  unagi_vx_ = cfg_.start_speed;
  unagi_vy_ = 0.0;
  unagi_omega_ = 0.0;
  elapsed_ = 0.0;
  flap_x_ = 1.0;
  flap_y_ = 0.0;
  finished_ = false;
  success_ = false;
  reason_.clear();
}

std::pair<std::vector<ObstacleData>, std::vector<std::pair<double, double>>>
GameWorld::generate_obstacles(int64_t layout_seed) const
{
  std::mt19937 rng(static_cast<std::mt19937::result_type>(layout_seed));
  const int count = std::max(0, cfg_.obstacle_count);
  if (count == 0) {
    return {};
  }

  const double start_x = std::max(1.5, cfg_.obstacle_start_x);
  const double end_x = cfg_.world_length - std::max(0.0, cfg_.obstacle_end_margin);
  const double usable = end_x - start_x;
  if (usable <= 0.0) {
    return {};
  }

  const double radius_min = std::max(0.05, std::min(cfg_.obstacle_radius_min, cfg_.obstacle_radius_max));
  const double radius_max = std::max(radius_min, cfg_.obstacle_radius_max);
  const double spacing = usable / static_cast<double>(count);
  const double jitter_fraction = std::clamp(cfg_.obstacle_x_jitter, 0.0, 0.45);
  const double path_center_limit = std::max(
    0.0,
    cfg_.arena_half_height - cfg_.unagi_radius - cfg_.obstacle_path_margin - 0.25);

  double safe_y = 0.0;
  std::vector<std::pair<ObstacleData, std::pair<double, double>>> paired;
  paired.reserve(static_cast<size_t>(count));

  for (int i = 0; i < count; ++i) {
    const double base_x = start_x + (static_cast<double>(i) + 0.5) * spacing;
    const double x = base_x + uniform(rng, -spacing * jitter_fraction, spacing * jitter_fraction);

    safe_y += uniform(rng, -cfg_.path_max_shift, cfg_.path_max_shift);
    safe_y = std::clamp(safe_y, -path_center_limit, path_center_limit);

    const double radius = uniform(rng, radius_min, radius_max);
    const double y_limit = std::max(0.0, cfg_.arena_half_height - radius - cfg_.obstacle_wall_margin);
    const double clearance = radius + cfg_.unagi_radius + cfg_.obstacle_path_margin;

    const double lower_hi = std::min(y_limit, safe_y - clearance);
    const double upper_lo = std::max(-y_limit, safe_y + clearance);
    const double lower_len = std::max(0.0, lower_hi + y_limit);
    const double upper_len = std::max(0.0, y_limit - upper_lo);

    double y = 0.0;
    if (lower_len > 0.0 || upper_len > 0.0) {
      const bool choose_lower = uniform(rng, 0.0, lower_len + upper_len) < lower_len;
      y = choose_lower ? uniform(rng, -y_limit, lower_hi) : uniform(rng, upper_lo, y_limit);
    } else {
      y = safe_y >= 0.0 ? -y_limit : y_limit;
    }

    paired.push_back({ObstacleData{x, y, radius}, {x, safe_y}});
  }

  std::sort(
    paired.begin(), paired.end(),
    [](const auto & a, const auto & b) {return a.first.x < b.first.x;});

  std::vector<ObstacleData> obstacles;
  std::vector<std::pair<double, double>> safe_path;
  obstacles.reserve(paired.size());
  safe_path.reserve(paired.size());
  for (const auto & item : paired) {
    obstacles.push_back(item.first);
    safe_path.push_back(item.second);
  }
  return {obstacles, safe_path};
}

std::pair<double, double> GameWorld::normalize_flap(double x, double y)
{
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return {1.0, 0.0};
  }
  const double mag = std::hypot(x, y);
  if (mag < 1e-6) {
    return {0.0, 0.0};
  }
  return {x / mag, y / mag};
}

void GameWorld::set_flap(double x, double y)
{
  const auto normalized = normalize_flap(x, y);
  flap_x_ = normalized.first;
  flap_y_ = normalized.second;
}

std::optional<std::string> GameWorld::collision_reason() const
{
  const double r = cfg_.unagi_radius;
  if (unagi_y_ + r >= cfg_.arena_half_height || unagi_y_ - r <= -cfg_.arena_half_height) {
    return std::string("wall_collision");
  }
  for (const auto & obs : obstacles_) {
    const double rr = r + obs.radius;
    const double dx = unagi_x_ - obs.x;
    const double dy = unagi_y_ - obs.y;
    if (dx * dx + dy * dy <= rr * rr) {
      return std::string("obstacle_collision");
    }
  }
  return std::nullopt;
}

void GameWorld::step(double dt, double time_limit)
{
  if (finished_ || dt <= 0.0) {
    return;
  }
  dt = std::min(dt, 0.05);
  elapsed_ += dt;

  const double c = std::cos(unagi_theta_);
  const double s = std::sin(unagi_theta_);
  const double fx = std::max(0.0, flap_x_);
  const double fy = std::clamp(flap_y_, -1.0, 1.0);
  const double thrust_force = cfg_.thrust * fx;
  const double lateral_force = cfg_.lateral_thrust * fy;

  double ax = (thrust_force * c + lateral_force * -s) / cfg_.mass;
  double ay = (thrust_force * s + lateral_force * c) / cfg_.mass;
  ax -= cfg_.linear_drag * unagi_vx_;
  ay -= cfg_.linear_drag * unagi_vy_;

  unagi_vx_ += ax * dt;
  unagi_vy_ += ay * dt;
  const double speed = std::hypot(unagi_vx_, unagi_vy_);
  if (speed > cfg_.max_speed) {
    const double scale = cfg_.max_speed / speed;
    unagi_vx_ *= scale;
    unagi_vy_ *= scale;
  }

  double angular_accel = (cfg_.steering_torque * fy) / cfg_.inertia;
  angular_accel -= cfg_.angular_drag * unagi_omega_;
  unagi_omega_ += angular_accel * dt;
  unagi_omega_ = std::clamp(unagi_omega_, -cfg_.max_omega, cfg_.max_omega);

  unagi_x_ += unagi_vx_ * dt;
  unagi_y_ += unagi_vy_ * dt;
  unagi_theta_ += unagi_omega_ * dt;
  unagi_theta_ = std::atan2(std::sin(unagi_theta_), std::cos(unagi_theta_));

  if (const auto collision = collision_reason()) {
    finish(false, *collision);
  } else if (unagi_x_ >= cfg_.world_length) {
    finish(true, "goal");
  } else if (time_limit > 0.0 && elapsed_ >= time_limit) {
    finish(false, "time_limit");
  }
}

void GameWorld::finish(bool success, const std::string & reason)
{
  finished_ = true;
  success_ = success;
  reason_ = reason;
}

int GameWorld::score() const
{
  const int distance_score = static_cast<int>(
    std::max(0.0, std::min(unagi_x_, cfg_.world_length)) * 10.0);
  const int time_bonus = success_ ? std::max(0, static_cast<int>(500.0 - elapsed_ * 8.0)) : 0;
  return distance_score + time_bonus;
}

}  // namespace unagi_game
