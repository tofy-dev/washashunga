#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/PWMSparkMax.h>

class ConveyorSubsystem: public frc2::SubsystemBase {
  public:
    void RunConveyor(double speed);

  private:
    const int conv_pwm_channel = 2;
    frc::PWMSparkMax m_conveyorMotor{conv_pwm_channel};
};