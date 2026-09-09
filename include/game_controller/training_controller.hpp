#ifndef GAME_CONTROLLER_TRAINING_CONTROLLER_HPP_
#define GAME_CONTROLLER_TRAINING_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

#include <std_msgs/msg/empty.hpp>

#include <ros2neuro_msgs/msg/neuro_control.hpp>
#include <ros2neuro_msgs/msg/neuro_event.hpp>

#include "game_controller/trial_sequence.hpp"

namespace game_controller {

// Must match ros2neuro_feedback_wheel's Wheel.h Events struct exactly.
// (Blink is never emitted by game_controller itself -- it comes from
// ros2neuro_artifact_blink -- but is listed here too so the two structs
// stay in sync.)
struct Events {
    static const int Start     = 1;
    static const int Fixation  = 786;
    static const int CFeedback = 781;
    static const int Hit       = 897;
    static const int Miss      = 898;
    static const int Off       = 32768;
    static const int Blink     = 1024;
};

struct Duration {
    int begin;
    int start;
    int fixation;
    int cue;
    int feedback_min;
    int feedback_max;
    int boom;
    int timeout;
    int timeout_on_rest;
    int iti;
    int end;
};

enum class Direction { Left = 0, Right, Forward, Timeout, None };
enum class Modality { Calibration, Evaluation };

// Calibration/evaluation orchestrator -- see package README.
class TrainingController : public rclcpp::Node {
public:
    TrainingController(void);

    bool configure(void);
    void run(void);

private:
    void setevent(int event);
    void publish_control(float probability);
    void sleep(int ms);

    void on_probability(const ros2neuro_msgs::msg::NeuroControl::SharedPtr msg);
    void on_neuro_event(const ros2neuro_msgs::msg::NeuroEvent::SharedPtr msg);
    float derive_position(float class_a, float class_b) const;

    Direction class2direction(int classid) const;
    int direction2classid(Direction dir) const;
    Direction is_target_hit(float input, Direction direction, int elapsed_ms, int duration_ms) const;

    rclcpp::Publisher<ros2neuro_msgs::msg::NeuroEvent>::SharedPtr event_pub_;
    rclcpp::Publisher<ros2neuro_msgs::msg::NeuroControl>::SharedPtr control_pub_;
    // Evaluation-only: tells ros2neuro_artifact_blink's blink_detector_node
    // to close any still-open window and stop publishing for good, so it
    // can never be left with an unmatched onset/offset because of a
    // shutdown-timing race once the whole launch is torn down afterwards.
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr protocol_ended_pub_;
    rclcpp::Subscription<ros2neuro_msgs::msg::NeuroControl>::SharedPtr probability_sub_;
    rclcpp::Subscription<ros2neuro_msgs::msg::NeuroEvent>::SharedPtr event_sub_;

    TrialSequence trialsequence_;

    std::vector<int> classes_;
    std::vector<double> thresholds_;
    Duration duration_{};
    Modality modality_ = Modality::Calibration;
    std::string probability_topic_;

    float current_input_ = 0.5f;
    bool has_new_input_ = false;
    bool blink_active_ = false;

    // Evaluation-only outcome tally, reported as a summary once the
    // protocol ends -- see run()'s "Protocol ended" log.
    int count_hit_ = 0;
    int count_miss_ = 0;
    int count_timeout_ = 0;

    static constexpr float kRateHz = 100.0f;
};

} // namespace game_controller

#endif
