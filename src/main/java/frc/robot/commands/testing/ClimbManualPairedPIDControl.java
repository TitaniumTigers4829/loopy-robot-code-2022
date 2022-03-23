package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import java.util.function.DoubleSupplier;

public class ClimbManualPairedPIDControl extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbSubsystem m_climbSubsystem;
  private final DoubleSupplier m_rightStick;

  /**
   * Creates ClimbManualIndependentControl.
   *
   * @param subsystem The climb subsystem used by this command.
   */
  public ClimbManualPairedPIDControl(ClimbSubsystem subsystem, DoubleSupplier rightStick) {
    m_climbSubsystem = subsystem;
    addRequirements(m_climbSubsystem);  // Use addRequirements() to declare subsystem dependencies.

    m_rightStick = rightStick;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_climbSubsystem.setDesiredLeftHookHeight(m_rightStick.getAsDouble() + 1);
    m_climbSubsystem.setDesiredRightHookHeight(m_rightStick.getAsDouble() + 1);
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

