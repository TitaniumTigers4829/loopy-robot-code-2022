package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbVertical extends InstantCommand {
  private ClimbSubsystem m_climb;
  public ClimbVertical(ClimbSubsystem climb){
    m_climb = climb;
    addRequirements(m_climb);
  }
  @Override
  public void initialize() {
    m_climb.setClimbVertical();
  }
}
