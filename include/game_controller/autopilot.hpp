#ifndef GAME_CONTROLLER_AUTOPILOT_HPP_
#define GAME_CONTROLLER_AUTOPILOT_HPP_

#include <random>

namespace game_controller {

// Fake-feedback signal generators for calibration mode, ported unchanged
// from ros2neuro_feedback_wheel's Autopilot/LinearPilot/SinePilot.
class Autopilot {
public:
    explicit Autopilot(float dt);
    virtual ~Autopilot(void);

    virtual void set(float start, float stop, int duration) = 0;
    virtual float step(void) = 0;

protected:
    float dt_;
    std::random_device rnddev_;
    std::mt19937 rndgen_;
};

class LinearPilot : public Autopilot {
public:
    explicit LinearPilot(float dt);
    ~LinearPilot(void);

    void set(float start, float stop, int duration) override;
    float step(void) override;

private:
    float step_;
};

class SinePilot : public Autopilot {
public:
    SinePilot(float dt, float frequency, float scale);
    ~SinePilot(void);

    void set(float start, float stop, int duration) override;
    float step(void) override;

private:
    float frequency_;
    float value_;
    float offset_;
    float amplitude_;
    float scale_;
    float t_;
    float sign_;
};

} // namespace game_controller

#endif
