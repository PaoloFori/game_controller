#include "game_controller/training_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

#include "game_controller/autopilot.hpp"

namespace game_controller {

TrainingController::TrainingController(void)
: rclcpp::Node("training_controller") {}

bool TrainingController::configure(void) {

    this->declare_parameter<std::string>("modality", "calibration");
    this->declare_parameter("classes", std::vector<int64_t>{773, 771});
    this->declare_parameter("trials", std::vector<int64_t>{10, 10});
    this->declare_parameter("thresholds", std::vector<double>{0.8, 0.2});

    this->declare_parameter<std::string>("control_topic", "/game_controller/control");
    this->declare_parameter<std::string>("event_topic", "/neuroevent");
    this->declare_parameter<std::string>("probability_topic", "/integrated/raw");

    this->declare_parameter("duration.begin", 5000);
    this->declare_parameter("duration.start", 1000);
    this->declare_parameter("duration.fixation", 2000);
    this->declare_parameter("duration.cue", 1000);
    this->declare_parameter("duration.feedback_min", 4000);
    this->declare_parameter("duration.feedback_max", 5500);
    this->declare_parameter("duration.boom", 1000);
    this->declare_parameter("duration.timeout", 10000);
    this->declare_parameter("duration.timeout_on_rest", 6000);
    this->declare_parameter("duration.iti", 100);
    this->declare_parameter("duration.end", 2000);

    std::string modality;
    this->get_parameter("modality", modality);
    if (modality == "calibration") {
        this->modality_ = Modality::Calibration;
    } else if (modality == "evaluation") {
        this->modality_ = Modality::Evaluation;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown modality '%s', expected 'calibration' or 'evaluation'", modality.c_str());
        return false;
    }

    std::vector<int64_t> classes64;
    this->get_parameter("classes", classes64);
    if (classes64.size() != 2 && classes64.size() != 3) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'classes' must contain 2 or 3 values");
        return false;
    }
    this->classes_.clear();
    for (auto v : classes64)
        this->classes_.push_back(static_cast<int>(v));

    std::vector<int64_t> trials64;
    this->get_parameter("trials", trials64);
    if (trials64.size() != classes64.size()) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'trials' must have the same length as 'classes'");
        return false;
    }

    this->get_parameter("thresholds", this->thresholds_);
    if (this->thresholds_.size() != 2) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'thresholds' must contain exactly 2 values");
        return false;
    }

    this->get_parameter("duration.begin", this->duration_.begin);
    this->get_parameter("duration.start", this->duration_.start);
    this->get_parameter("duration.fixation", this->duration_.fixation);
    this->get_parameter("duration.cue", this->duration_.cue);
    this->get_parameter("duration.feedback_min", this->duration_.feedback_min);
    this->get_parameter("duration.feedback_max", this->duration_.feedback_max);
    this->get_parameter("duration.boom", this->duration_.boom);
    this->get_parameter("duration.timeout", this->duration_.timeout);
    this->get_parameter("duration.timeout_on_rest", this->duration_.timeout_on_rest);
    this->get_parameter("duration.iti", this->duration_.iti);
    this->get_parameter("duration.end", this->duration_.end);

    int mindur_active, maxdur_active, mindur_rest, maxdur_rest;
    if (this->modality_ == Modality::Calibration) {
        mindur_active = this->duration_.feedback_min;
        maxdur_active = this->duration_.feedback_max;
        mindur_rest = this->duration_.feedback_min;
        maxdur_rest = this->duration_.feedback_max;
    } else {
        mindur_active = this->duration_.timeout;
        maxdur_active = this->duration_.timeout;
        mindur_rest = this->duration_.timeout_on_rest;
        maxdur_rest = this->duration_.timeout_on_rest;
    }

    this->trialsequence_.addclass(this->classes_.at(0), static_cast<int>(trials64.at(0)), mindur_active, maxdur_active);
    this->trialsequence_.addclass(this->classes_.at(1), static_cast<int>(trials64.at(1)), mindur_active, maxdur_active);
    if (this->classes_.size() == 3)
        this->trialsequence_.addclass(this->classes_.at(2), static_cast<int>(trials64.at(2)), mindur_rest, maxdur_rest);

    RCLCPP_INFO(this->get_logger(), "Trials: %d", this->trialsequence_.size());

    std::string control_topic, event_topic;
    this->get_parameter("control_topic", control_topic);
    this->get_parameter("event_topic", event_topic);
    this->get_parameter("probability_topic", this->probability_topic_);

    this->control_pub_ = this->create_publisher<ros2neuro_msgs::msg::NeuroControl>(control_topic, 10);
    this->event_pub_ = this->create_publisher<ros2neuro_msgs::msg::NeuroEvent>(event_topic, 10);

    if (this->modality_ == Modality::Evaluation) {
        this->probability_sub_ = this->create_subscription<ros2neuro_msgs::msg::NeuroControl>(
            this->probability_topic_,
            10,
            std::bind(&TrainingController::on_probability, this, std::placeholders::_1));
    }

    return true;
}

void TrainingController::run(void) {

    rclcpp::Rate r(kRateHz);

    LinearPilot linearpilot(1000.0f / kRateHz);
    SinePilot sinepilot(1000.0f / kRateHz, 0.25f, 0.5f);
    Autopilot* autopilot = nullptr;

    RCLCPP_INFO(this->get_logger(), "Protocol started");

    this->sleep(this->duration_.begin);
    if (!rclcpp::ok()) {
        return;
    }

    for (auto it = this->trialsequence_.begin(); it != this->trialsequence_.end(); ++it) {

        const int trialnumber = static_cast<int>(it - this->trialsequence_.begin()) + 1;
        const int trialclass = it->classid;
        const int trialduration = it->duration;
        const Direction trialdirection = this->class2direction(trialclass);

        const double trialthreshold =
            (trialdirection == Direction::Left) ? this->thresholds_.at(0) : this->thresholds_.at(1);

        Direction targethit = Direction::None;

        if (this->modality_ == Modality::Calibration) {
            autopilot = (trialdirection == Direction::Forward)
                ? static_cast<Autopilot*>(&sinepilot)
                : static_cast<Autopilot*>(&linearpilot);
            autopilot->set(0.5f, static_cast<float>(trialthreshold), trialduration);
        }

        this->setevent(Events::Start);
        this->sleep(this->duration_.start);
        if (!rclcpp::ok()) {
            return;
        }
        this->setevent(Events::Start + Events::Off);

        this->setevent(Events::Fixation);
        this->sleep(this->duration_.fixation);
        if (!rclcpp::ok()) {
            return;
        }
        this->setevent(Events::Fixation + Events::Off);

        this->setevent(trialclass);
        this->sleep(this->duration_.cue);
        if (!rclcpp::ok()) {
            return;
        }

        rclcpp::spin_some(this->shared_from_this());

        this->setevent(trialclass + Events::Off);
        this->setevent(Events::CFeedback);

        this->has_new_input_ = false;
        this->current_input_ = 0.5f;

        if (this->modality_ == Modality::Evaluation) {
            // A subscription's ROS queue can still hold up to its depth in
            // stale messages (e.g. the previous trial's saturated buffer,
            // from before the reset_event-triggered integrator reset this
            // CFeedback just caused) -- resetting current_input_/
            // has_new_input_ above doesn't clear that backlog, and the
            // first spin_some() below would otherwise deliver it, feeding
            // is_target_hit() a leftover value instead of this trial's
            // actual (neutral, just-reset) starting point. Recreating the
            // subscription discards the backlog outright.
            this->probability_sub_.reset();
            this->probability_sub_ = this->create_subscription<ros2neuro_msgs::msg::NeuroControl>(
                this->probability_topic_,
                10,
                std::bind(&TrainingController::on_probability, this, std::placeholders::_1));
        }

        RCLCPP_INFO(
            this->get_logger(), "Trial %d/%d (class: %d | duration: %d ms)",
            trialnumber, this->trialsequence_.size(), trialclass, trialduration);

        const auto trial_start = std::chrono::steady_clock::now();

        while (rclcpp::ok() && targethit == Direction::None) {

            if (this->modality_ == Modality::Calibration) {
                this->current_input_ += autopilot->step();
                this->publish_control(this->current_input_);
            } else {
                rclcpp::spin_some(this->shared_from_this());
                if (this->has_new_input_) {
                    this->publish_control(this->current_input_);
                    this->has_new_input_ = false;
                }
            }

            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - trial_start).count();

            targethit = this->is_target_hit(
                this->current_input_, trialdirection, static_cast<int>(elapsed_ms), trialduration);

            r.sleep();
        }

        this->setevent(Events::CFeedback + Events::Off);

        // Give the wheel a comfortable margin to have already processed the
        // final control value (the one that put it on the threshold) before
        // the outcome events below arrive on a separate topic -- control
        // and neuroevent have no cross-topic delivery-order guarantee, and
        // without this gap they can be published close enough together
        // that the wheel's boom appears to show up before it visually
        // reaches the threshold.
        this->sleep(20);
        if (!rclcpp::ok()) {
            return;
        }

        //const int reached_classid = this->direction2classid(targethit);
        //if (reached_classid >= 0)
        //    this->setevent(reached_classid + Events::Command);

        const int outcome = (trialdirection == targethit) ? Events::Hit : Events::Miss;
        this->setevent(outcome);
        this->sleep(this->duration_.boom);
        if (!rclcpp::ok()) {
            return;
        }
        this->setevent(outcome + Events::Off);

        // Explicitly publish a centered control value once the outcome boom
        // is hidden, so the wheel resets to its starting position exactly
        // like it would for a real 0.5 (i.e. "undecided") classifier output.
        this->publish_control(0.5f);

        this->sleep(this->duration_.iti);
        if (!rclcpp::ok()) {
            return;
        }
    }

    this->sleep(this->duration_.end);
    if (!rclcpp::ok()) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Protocol ended");
}

void TrainingController::on_probability(const ros2neuro_msgs::msg::NeuroControl::SharedPtr msg) {

    if (msg->values.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Received NeuroControl with fewer than 2 values, ignoring");
        return;
    }

    this->current_input_ = this->derive_position(msg->values.at(0), msg->values.at(1));
    this->has_new_input_ = true;
}

float TrainingController::derive_position(float class_a, float class_b) const {
    const float total = class_a + class_b;
    if (total <= 0.0f)
        return 0.5f;
    return class_b / total;
}

void TrainingController::setevent(int event) {
    ros2neuro_msgs::msg::NeuroEvent msg;
    msg.header.stamp = this->now();
    msg.event_id = event;
    this->event_pub_->publish(msg);
}

void TrainingController::publish_control(float probability) {
    ros2neuro_msgs::msg::NeuroControl msg;
    msg.header.stamp = this->now();
    msg.source = this->get_name();
    msg.values = {probability};
    this->control_pub_->publish(msg);
}

void TrainingController::sleep(int ms) {
    rclcpp::sleep_for(std::chrono::milliseconds(ms));
}

Direction TrainingController::class2direction(int classid) const {

    auto it = std::find(this->classes_.begin(), this->classes_.end(), classid);

    if (it == this->classes_.end())
        return Direction::None;

    return static_cast<Direction>(it - this->classes_.begin());
}

int TrainingController::direction2classid(Direction dir) const {

    const std::size_t idx = static_cast<std::size_t>(dir);

    if (idx < this->classes_.size())
        return this->classes_.at(idx);

    return -1;
}

Direction TrainingController::is_target_hit(
    float input, Direction direction, int elapsed_ms, int duration_ms) const {

    if (input >= this->thresholds_.at(0))
        return Direction::Left;

    if (input <= this->thresholds_.at(1))
        return Direction::Right;

    if (direction == Direction::Forward && elapsed_ms >= duration_ms)
        return Direction::Forward;

    if (elapsed_ms >= duration_ms)
        return Direction::Timeout;

    return Direction::None;
}

} // namespace game_controller

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<game_controller::TrainingController>();

    if (!node->configure()) {
        RCLCPP_ERROR(node->get_logger(), "TrainingController configuration failed");
        rclcpp::shutdown();
        return 1;
    }

    try {
        node->run();
    } catch (const rclcpp::exceptions::RCLError& ex) {
        // rclcpp::ok() is checked throughout run(), but shutdown happens
        // asynchronously on a signal-handling thread -- a call like
        // spin_some() can still race past the check and throw if shutdown
        // lands in that gap. Treat it as a normal shutdown, not a crash.
        RCLCPP_WARN(node->get_logger(), "run() interrupted by shutdown: %s", ex.what());
    }

    rclcpp::shutdown();
    return 0;
}
