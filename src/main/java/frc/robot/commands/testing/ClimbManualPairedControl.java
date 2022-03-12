package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import java.util.function.DoubleSupplier;

public class ClimbManualPairedControl extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbSubsystem m_climbSubsystem;

  private final DoubleSupplier m_rightStick;

  /**
   * Creates ClimbManualIndependentControl.
   *
   * @param subsystem The climb subsystem used by this command.
   */
  public ClimbManualPairedControl(ClimbSubsystem subsystem, DoubleSupplier rightStick) {
    m_climbSubsystem = subsystem;
    m_rightStick = rightStick;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_climbSubsystem.setLeftMotorOutputManual(m_rightStick.getAsDouble());
    m_climbSubsystem.setRightMotorOutputManual(m_rightStick.getAsDouble());
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

