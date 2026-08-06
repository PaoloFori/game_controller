#include "game_controller/trial_sequence.hpp"

#include <algorithm>

namespace game_controller {

TrialSequence::TrialSequence(void) {
    this->rndgen_.seed(this->rnddev_());
}

TrialSequence::~TrialSequence(void) {}

bool TrialSequence::addclass(int classid, int ntrials, int mindur, int maxdur) {

    auto is_already_present = [classid](const Trial& t) { return t.classid == classid; };
    auto it = std::find_if(this->sequence_.begin(), this->sequence_.end(), is_already_present);

    if (it != this->sequence_.end())
        return false;

    auto intdis = std::uniform_int_distribution<int>(mindur, maxdur);
    Trial trial;
    trial.classid = classid;

    for (int i = 0; i < ntrials; ++i) {
        trial.duration = intdis(this->rndgen_);
        this->sequence_.push_back(trial);
    }

    std::shuffle(this->sequence_.begin(), this->sequence_.end(), this->rndgen_);

    return true;
}

bool TrialSequence::addclass(int classid, int ntrials, int dur) {
    return this->addclass(classid, ntrials, dur, dur);
}

int TrialSequence::size(void) {
    return static_cast<int>(this->sequence_.size());
}

std::vector<Trial>::iterator TrialSequence::begin(void) {
    return this->sequence_.begin();
}

std::vector<Trial>::iterator TrialSequence::end(void) {
    return this->sequence_.end();
}

} // namespace game_controller
