package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbBottomPositon extends CommandBase {

  private final ClimbSubsystem m_climb;

  public ClimbBottomPositon(ClimbSubsystem climb) {
    m_climb = climb;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climb.setLeftHookToBottomPos();
    m_climb.setRightHookToBottomPos();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_climb.getIsLeftLimitSwitchPressed() && m_climb.getIsRightLimitSwitchPressed());
  }
}
