#ifndef GAME_CONTROLLER_TRIAL_SEQUENCE_HPP_
#define GAME_CONTROLLER_TRIAL_SEQUENCE_HPP_

#include <random>
#include <vector>

namespace game_controller {

struct Trial {
    int classid;
    int duration;
};

// Shuffled sequence of (class id, duration) trials, ported unchanged from
// ros2neuro_feedback_wheel's TrialSequence.
class TrialSequence {
public:
    TrialSequence(void);
    ~TrialSequence(void);

    bool addclass(int classid, int ntrials, int mindur, int maxdur);
    bool addclass(int classid, int ntrials, int dur);

    int size(void);

    std::vector<Trial>::iterator begin(void);
    std::vector<Trial>::iterator end(void);

private:
    std::vector<Trial> sequence_;

    std::random_device rnddev_;
    std::mt19937 rndgen_;
};

} // namespace game_controller

#endif
