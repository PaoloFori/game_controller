#include "game_controller/autopilot.hpp"

#include <cmath>

namespace game_controller {

Autopilot::Autopilot(float dt) {
    this->dt_ = dt;
}
Autopilot::~Autopilot(void) {}

LinearPilot::LinearPilot(float dt) : Autopilot(dt) {}
LinearPilot::~LinearPilot(void) {}

void LinearPilot::set(float start, float stop, int duration) {
    float distance = stop - std::fabs(start);
    this->step_ = distance / (static_cast<float>(duration) / this->dt_);
}

float LinearPilot::step(void) {
    return this->step_;
}

SinePilot::SinePilot(float dt, float frequency, float scale) : Autopilot(dt) {
    this->frequency_ = frequency;
    this->scale_ = scale;
}
SinePilot::~SinePilot(void) {}

void SinePilot::set(float start, float stop, int duration) {
    (void)duration;

    float limit = this->scale_ * (stop - std::fabs(start));
    auto realdis = std::uniform_real_distribution<float>(0.0f, limit);
    auto intdis = std::uniform_int_distribution<int>(0, 1);

    this->amplitude_ = realdis(this->rndgen_);
    this->sign_ = intdis(this->rndgen_) ? -1 : 1;
    this->offset_ = start;
    this->value_ = start;
    this->t_ = 0.0f;
}

float SinePilot::step(void) {
    float value = this->amplitude_ * (std::sin(2.0f * M_PI * this->frequency_ * this->t_ / 1000.0f)) + this->offset_;
    float step = value - this->value_;

    this->value_ = value;
    this->t_ = this->t_ + this->dt_;

    return this->sign_ * step;
}

} // namespace game_controller
