#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "unagi_game/action/play_round.hpp"
#include "unagi_game/game_world.hpp"
#include "unagi_game/msg/game_state.hpp"
#include "unagi_game/msg/obstacle.hpp"

using namespace std::chrono_literals;

namespace unagi_game
{

class PhysicsNode : public rclcpp::Node
{
public:
  using PlayRound = unagi_game::action::PlayRound;
  using GoalHandlePlayRound = rclcpp_action::ServerGoalHandle<PlayRound>;

  PhysicsNode()
  : Node("unagi_game_physics"), world_(make_config())
  {
    default_time_limit_ = declare_parameter<double>("default_time_limit_sec", 45.0);
    time_limit_ = default_time_limit_;
    active_ = declare_parameter<bool>("auto_start", true);
    paused_ = false;

    state_pub_ = create_publisher<msg::GameState>("/game/state", 10);
    event_pub_ = create_publisher<std_msgs::msg::String>("/game/event", 10);
    flap_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      "/game/flap_direction", 10,
      [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        world_.set_flap(msg->x, msg->y);
      });

    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "/game/reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        uint32_t round;
        int64_t seed;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          world_.reset();
          time_limit_ = default_time_limit_;
          active_ = true;
          paused_ = false;
          round = world_.round_index();
          seed = world_.layout_seed();
        }
        response->success = true;
        response->message = "Round " + std::to_string(round) +
          " reset with layout seed " + std::to_string(seed) + ".";
        publish_event("reset:round=" + std::to_string(round) + ":seed=" + std::to_string(seed));
      });

    pause_srv_ = create_service<std_srvs::srv::SetBool>(
      "/game/pause",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          paused_ = request->data;
        }
        response->success = true;
        response->message = request->data ? "paused" : "resumed";
        publish_event(response->message);
      });

    action_server_ = rclcpp_action::create_server<PlayRound>(
      this,
      "/game/play_round",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const PlayRound::Goal>) {
        std::lock_guard<std::mutex> lock(mutex_);
        return action_running_ ? rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandlePlayRound>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandlePlayRound> goal_handle) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          action_running_ = true;
        }
        std::thread([this, goal_handle]() {execute_round(goal_handle);}).detach();
      });

    const double hz = std::max(1.0, declare_parameter<double>("physics_hz", 60.0));
    dt_ = 1.0 / hz;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      [this]() {on_tick();});

    RCLCPP_INFO(
      get_logger(), "Physics ready: %.1f Hz, world=%.1f, auto_start=%s",
      hz, world_.config().world_length, active_ ? "true" : "false");
  }

private:
  PhysicsConfig make_config()
  {
    PhysicsConfig cfg;
    cfg.world_length = declare_parameter<double>("world_length", 55.0);
    cfg.arena_half_height = declare_parameter<double>("arena_half_height", 4.0);
    cfg.unagi_radius = declare_parameter<double>("unagi_radius", 0.24);
    cfg.obstacle_count = declare_parameter<int>("obstacle_count", 18);
    cfg.randomize_obstacles_each_round = declare_parameter<bool>("randomize_obstacles_each_round", true);
    cfg.obstacle_seed = declare_parameter<int64_t>("obstacle_seed", -1);
    cfg.obstacle_radius_min = declare_parameter<double>("obstacle_radius_min", 0.45);
    cfg.obstacle_radius_max = declare_parameter<double>("obstacle_radius_max", 0.85);
    cfg.obstacle_x_jitter = declare_parameter<double>("obstacle_x_jitter", 0.30);
    cfg.obstacle_path_margin = declare_parameter<double>("obstacle_path_margin", 0.55);
    cfg.path_max_shift = declare_parameter<double>("path_max_shift", 0.85);
    cfg.thrust = declare_parameter<double>("thrust", 8.0);
    cfg.steering_torque = declare_parameter<double>("steering_torque", 4.5);
    return cfg;
  }

  void publish_event(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    event_pub_->publish(msg);
  }

  void execute_round(const std::shared_ptr<GoalHandlePlayRound> goal_handle)
  {
    uint32_t round;
    int64_t seed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double requested = goal_handle->get_goal()->time_limit_sec;
      time_limit_ = requested > 0.0 ? requested : default_time_limit_;
      world_.reset();
      active_ = true;
      paused_ = false;
      round = world_.round_index();
      seed = world_.layout_seed();
    }
    publish_event("round_started:round=" + std::to_string(round) + ":seed=" + std::to_string(seed));

    rclcpp::Rate feedback_rate(20.0);
    std::string fallback_reason = "shutdown";
    while (rclcpp::ok()) {
      auto feedback = std::make_shared<PlayRound::Feedback>();
      bool finished;
      bool success;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        finished = world_.finished();
        success = world_.success();
        feedback->elapsed_sec = static_cast<float>(world_.elapsed());
        feedback->distance = static_cast<float>(world_.unagi_x());
        feedback->score = world_.score();
      }
      goal_handle->publish_feedback(feedback);

      if (goal_handle->is_canceling()) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          active_ = false;
          world_.finish(false, "cancelled");
        }
        auto result = make_action_result("cancelled");
        goal_handle->canceled(result);
        fallback_reason = "cancelled";
        finish_action(fallback_reason);
        return;
      }

      if (finished) {
        auto result = make_action_result(fallback_reason);
        if (success) {
          goal_handle->succeed(result);
        } else {
          goal_handle->abort(result);
        }
        finish_action(result->reason);
        return;
      }
      feedback_rate.sleep();
    }

    finish_action(fallback_reason);
  }

  std::shared_ptr<PlayRound::Result> make_action_result(const std::string & fallback)
  {
    auto result = std::make_shared<PlayRound::Result>();
    std::lock_guard<std::mutex> lock(mutex_);
    result->success = world_.success();
    result->score = world_.score();
    result->distance = static_cast<float>(world_.unagi_x());
    result->reason = world_.reason().empty() ? fallback : world_.reason();
    active_ = false;
    return result;
  }

  void finish_action(const std::string & reason)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      action_running_ = false;
    }
    publish_event("round_finished:" + reason);
  }

  void on_tick()
  {
    msg::GameState state;
    std::string finished_reason;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool was_finished = world_.finished();
      if (active_ && !paused_ && !world_.finished()) {
        world_.step(dt_, time_limit_);
        if (!was_finished && world_.finished() && !action_running_) {
          active_ = false;
          finished_reason = world_.reason();
        }
      }
      state = make_state_message_locked();
    }
    state_pub_->publish(state);
    if (!finished_reason.empty()) {
      publish_event("round_finished:" + finished_reason);
    }
  }

  msg::GameState make_state_message_locked() const
  {
    msg::GameState out;
    out.stamp = now();
    out.active = active_;
    out.paused = paused_;
    out.finished = world_.finished();
    out.success = world_.success();
    out.reason = world_.reason();
    out.elapsed_sec = static_cast<float>(world_.elapsed());
    out.score = world_.score();
    out.round_index = world_.round_index();
    out.layout_seed = world_.layout_seed();
    out.world_length = static_cast<float>(world_.config().world_length);
    out.arena_half_height = static_cast<float>(world_.config().arena_half_height);
    out.unagi_x = static_cast<float>(world_.unagi_x());
    out.unagi_y = static_cast<float>(world_.unagi_y());
    out.unagi_theta = static_cast<float>(world_.unagi_theta());
    out.unagi_vx = static_cast<float>(world_.unagi_vx());
    out.unagi_vy = static_cast<float>(world_.unagi_vy());
    out.unagi_omega = static_cast<float>(world_.unagi_omega());
    out.flap_x = static_cast<float>(world_.flap_x());
    out.flap_y = static_cast<float>(world_.flap_y());
    out.obstacles.reserve(world_.obstacles().size());
    for (const auto & item : world_.obstacles()) {
      msg::Obstacle obstacle;
      obstacle.x = static_cast<float>(item.x);
      obstacle.y = static_cast<float>(item.y);
      obstacle.radius = static_cast<float>(item.radius);
      out.obstacles.push_back(obstacle);
    }
    return out;
  }

  mutable std::mutex mutex_;
  GameWorld world_;
  double default_time_limit_{45.0};
  double time_limit_{45.0};
  double dt_{1.0 / 60.0};
  bool active_{true};
  bool paused_{false};
  bool action_running_{false};

  rclcpp::Publisher<msg::GameState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr flap_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_srv_;
  rclcpp_action::Server<PlayRound>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace unagi_game

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<unagi_game::PhysicsNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
