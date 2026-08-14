#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "unagi_game/msg/game_state.hpp"

namespace unagi_game
{

class RendererNode : public rclcpp::Node
{
public:
  RendererNode()
  : Node("unagi_game_renderer")
  {
    width_ = declare_parameter<int>("width", 960);
    height_ = declare_parameter<int>("height", 540);
    visible_x_ = declare_parameter<double>("meters_visible_x", 15.0);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/game/image", 3);
    state_sub_ = create_subscription<msg::GameState>(
      "/game/state", 3,
      [this](const msg::GameState::SharedPtr state) {on_state(*state);});
  }

private:
  cv::Point world_to_px(double x, double y, double camera_x, double half_h) const
  {
    const double left = camera_x - visible_x_ * 0.25;
    const int px = static_cast<int>((x - left) / visible_x_ * width_);
    const int py = static_cast<int>((half_h - y) / (2.0 * half_h) * height_);
    return {px, py};
  }

  static std::string reason_text(std::string text)
  {
    std::replace(text.begin(), text.end(), '_', ' ');
    std::transform(text.begin(), text.end(), text.begin(),
      [](unsigned char c) {return static_cast<char>(std::toupper(c));});
    return text;
  }

  void on_state(const msg::GameState & state)
  {
    cv::Mat img(height_, width_, CV_8UC3, cv::Scalar(18, 28, 38));
    const double camera_x = std::max(visible_x_ * 0.25, static_cast<double>(state.unagi_x));
    const double half_h = std::max(0.5, static_cast<double>(state.arena_half_height));

    for (int i = 0; i < 9; ++i) {
      const double y = -half_h + (2.0 * half_h * i / 8.0);
      const int py = world_to_px(camera_x, y, camera_x, half_h).y;
      cv::line(img, {0, py}, {width_, py}, cv::Scalar(31, 48, 60), 1, cv::LINE_AA);
    }

    cv::line(img, {0, 1}, {width_, 1}, cv::Scalar(95, 120, 135), 3);
    cv::line(img, {0, height_ - 2}, {width_, height_ - 2}, cv::Scalar(95, 120, 135), 3);

    const int gx = world_to_px(state.world_length, 0.0, camera_x, half_h).x;
    if (gx >= -30 && gx <= width_ + 30) {
      cv::line(img, {gx, 0}, {gx, height_}, cv::Scalar(80, 210, 120), 5);
      cv::putText(img, "GOAL", {gx - 24, 32}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
        cv::Scalar(170, 255, 190), 2, cv::LINE_AA);
    }

    const double px_per_m = static_cast<double>(width_) / visible_x_;
    for (const auto & obs : state.obstacles) {
      const cv::Point p = world_to_px(obs.x, obs.y, camera_x, half_h);
      const int rr = std::max(2, static_cast<int>(obs.radius * px_per_m));
      if (p.x + rr < 0 || p.x - rr > width_) {
        continue;
      }
      cv::circle(img, p, rr, cv::Scalar(70, 82, 108), -1, cv::LINE_AA);
      cv::circle(img, p, rr, cv::Scalar(150, 165, 190), 2, cv::LINE_AA);
      cv::circle(img, {p.x - rr / 4, p.y - rr / 4}, std::max(2, rr / 7),
        cv::Scalar(95, 108, 135), -1, cv::LINE_AA);
    }

    const cv::Point center = world_to_px(state.unagi_x, state.unagi_y, camera_x, half_h);
    const double theta = state.unagi_theta;
    const int head_len = static_cast<int>(0.48 * px_per_m);
    const int head_w = static_cast<int>(0.26 * px_per_m);
    const cv::Point2d fwd(std::cos(theta), -std::sin(theta));
    const cv::Point2d left(std::sin(theta), std::cos(theta));
    const cv::Point2d c(center.x, center.y);
    const cv::Point2d nose = c + fwd * (head_len * 0.55);
    const cv::Point2d back = c - fwd * (head_len * 0.45);

    std::vector<cv::Point> head{
      cv::Point(static_cast<int>(nose.x), static_cast<int>(nose.y)),
      cv::Point(static_cast<int>((back + left * head_w).x), static_cast<int>((back + left * head_w).y)),
      cv::Point(static_cast<int>((back - left * head_w).x), static_cast<int>((back - left * head_w).y))};
    cv::fillConvexPoly(img, head, cv::Scalar(225, 235, 245), cv::LINE_AA);
    cv::polylines(img, head, true, cv::Scalar(120, 190, 225), 2, cv::LINE_AA);

    std::vector<cv::Point> tail;
    tail.reserve(16);
    const cv::Point2d tail_start = back - fwd * 2.0;
    const double phase = state.elapsed_sec * 16.0;
    const double steer = state.flap_y;
    for (int i = 0; i < 16; ++i) {
      const double t = static_cast<double>(i) / 15.0;
      const cv::Point2d base = tail_start - fwd * (t * 1.75 * px_per_m);
      const double wave = std::sin(phase - t * 8.0) * (4.0 + 10.0 * t);
      const double bias = steer * (10.0 + 22.0 * t);
      const cv::Point2d p = base + left * (wave + bias);
      tail.emplace_back(static_cast<int>(p.x), static_cast<int>(p.y));
    }
    cv::polylines(img, tail, false, cv::Scalar(180, 220, 240), 3, cv::LINE_AA);

    const double speed = std::hypot(state.unagi_vx, state.unagi_vy);
    const std::string status = state.paused ? "PAUSED" : (state.finished ? "FINISHED" : "RUN");
    std::ostringstream hud;
    hud << status << "   round " << std::setw(2) << std::setfill('0') << state.round_index
        << "   score " << std::setw(4) << state.score
        << "   time " << std::fixed << std::setprecision(1) << std::setw(5) << state.elapsed_sec
        << "s   speed " << std::setprecision(1) << speed;
    cv::rectangle(img, {12, 10}, {700, 48}, cv::Scalar(10, 16, 24), -1);
    cv::putText(img, hud.str(), {24, 36}, cv::FONT_HERSHEY_SIMPLEX, 0.65,
      cv::Scalar(235, 240, 245), 2, cv::LINE_AA);
    cv::putText(img, "A/D: flap left/right   W: straight   R: reset   P: pause",
      {24, height_ - 18}, cv::FONT_HERSHEY_SIMPLEX, 0.52,
      cv::Scalar(205, 215, 225), 1, cv::LINE_AA);

    if (state.finished) {
      const std::string text = state.success ? "GOAL!" : reason_text(state.reason);
      int baseline = 0;
      const cv::Size size = cv::getTextSize(text, cv::FONT_HERSHEY_DUPLEX, 1.3, 2, &baseline);
      cv::putText(img, text, {(width_ - size.width) / 2, height_ / 2},
        cv::FONT_HERSHEY_DUPLEX, 1.3, cv::Scalar(245, 245, 245), 2, cv::LINE_AA);
    }

    sensor_msgs::msg::Image out;
    out.header.stamp = state.stamp;
    out.header.frame_id = "game_screen";
    out.height = static_cast<uint32_t>(height_);
    out.width = static_cast<uint32_t>(width_);
    out.encoding = "bgr8";
    out.is_bigendian = 0;
    out.step = static_cast<uint32_t>(width_ * 3);
    const size_t bytes = img.total() * img.elemSize();
    out.data.assign(img.data, img.data + bytes);
    image_pub_->publish(out);
  }

  int width_{960};
  int height_{540};
  double visible_x_{15.0};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<msg::GameState>::SharedPtr state_sub_;
};

}  // namespace unagi_game

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<unagi_game::RendererNode>());
  rclcpp::shutdown();
  return 0;
}
