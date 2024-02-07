#include "subsystems/ConveyorSubsystem.h"

void ConveyorSubsystem::RunConveyor(double speed) {
    m_conveyorMotor.Set(speed * 0.3);
}