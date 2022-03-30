package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbFullExtension extends CommandBase {

  private final ClimbSubsystem m_climb;

  public ClimbFullExtension(ClimbSubsystem climb) {
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
    m_climb.setLeftHookToFullExtension();
    m_climb.setRightHookToFullExtension();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.setLeftMotorOutputManual(0);
    m_climb.setRightMotorOutputManual(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((m_climb.getLeftHookHeight() >= ClimbConstants.kClimbMaxHeight + (
        ClimbConstants.kClimbMaxPosConfirmationExtraHeight / 2)) &&
        (m_climb.getRightHookHeight() >= ClimbConstants.kClimbMaxHeight + (
            ClimbConstants.kClimbMaxPosConfirmationExtraHeight / 2)));
  }
}
