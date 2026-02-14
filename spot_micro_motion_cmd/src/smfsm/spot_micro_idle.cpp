#include "spot_micro_idle.h"
#include "spot_micro_motion_cmd.h"
#include "spot_micro_transition_stand.h"

SpotMicroIdleState::SpotMicroIdleState() {
  // Construcotr, doesn't need to do anything, for now...
  //std::cout << "SpotMicroIdleState Ctor" << std::endl;
}

SpotMicroIdleState::~SpotMicroIdleState() {
  //std::cout << "SpotMicroIdleState Dtor" << std::endl;
}

void SpotMicroIdleState::handleInputCommands(const smk::BodyState& body_state,
                                             const SpotMicroNodeConfig& smnc,
                                             const Command& cmd,
                                             SpotMicroMotionCmd* smmc,
                                             smk::BodyState* body_state_cmd_) {
  if (smnc.debug_mode) {
    std::cout << "In Spot Micro Idle State" << std::endl;
  }
  
  // Check if stand command issued, if so, transition to stand state
  if (cmd.getStandCmd() == true) {
    changeState(smmc, std::make_unique<SpotMicroTransitionStandState>());
  
  } else {
    // Hold lie-down stance (low, relaxed pose) to avoid high current spike
    // when transitioning to stand. PWM 0 (freewheel) would cause all servos
    // to drive at once on stand command.
    smmc->setServoCommandMessageData();
    smmc->publishServoProportionalCommand();
  }

}

