package frc.robot.commands.testing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;
import java.util.function.DoubleSupplier;

public class setClimbToPos extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbSubsystem m_climbSubsystem;

  /**
   * Creates ClimbManualIndependentControl.
   *
   * @param subsystem The climb subsystem used by this command.
   */
  public setClimbToPos(ClimbSubsystem subsystem) {
    m_climbSubsystem = subsystem;
    addRequirements(m_climbSubsystem);  // Use addRequirements() to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("desired height", 1.2);
  }

  @Override
  public void execute() {
    m_climbSubsystem.setDesiredLeftHookHeight(SmartDashboard.getNumber("desired height", 1.2));
  }

  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.setLeftMotorOutputManual(0);
    m_climbSubsystem.setRightMotorOutputManual(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

