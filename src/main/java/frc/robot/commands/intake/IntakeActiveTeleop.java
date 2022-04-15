package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.BooleanSupplier;

public class IntakeActiveTeleop extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intakeSubsystem;

  /**
   * Creates IntakeActiveTeleop. By default, a "while held" command.
   *
   * @param m_intakeSubsystem The climb subsystem used by this command.
   */
  public IntakeActiveTeleop(IntakeSubsystem m_intakeSubsystem) {
    this.m_intakeSubsystem = m_intakeSubsystem;
    addRequirements(m_intakeSubsystem);  // Use addRequirements() to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    m_intakeSubsystem.setSolenoidDeployed();
    m_intakeSubsystem.setMotorFullPowerIn();
  }

  @Override
  public void execute() {
    m_intakeSubsystem.setMotorFullPowerIn();
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setMotorStopped();
    m_intakeSubsystem.setSolenoidRetracted();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
