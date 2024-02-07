#include "commands/RunConveyorCmd.h"

RunConveyorCmd::RunConveyorCmd(ConveyorSubsystem *subsystem,
                               std::function<double()> speedFcn)
    : m_subsystem{subsystem}, speedFcn{speedFcn} {
  AddRequirements(m_subsystem);
}

void RunConveyorCmd::Execute() {
  double speed = speedFcn();
  m_subsystem->RunConveyor(speed);
}