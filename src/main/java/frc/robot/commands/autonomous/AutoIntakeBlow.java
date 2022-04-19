package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeBlow extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intakeSubsystem;

  /**
   * Creates AutoIntakeBlow. By default, a "while held" command.
   *
   * @param m_intakeSubsystem The climb subsystem used by this command.
   */
  public AutoIntakeBlow(IntakeSubsystem m_intakeSubsystem) {
    this.m_intakeSubsystem = m_intakeSubsystem;
    addRequirements(m_intakeSubsystem);  // Use addRequirements() to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    m_intakeSubsystem.setSolenoidDeployed();
    m_intakeSubsystem.setMotorFullPowerOut();
  }

  @Override
  public void execute() {
    m_intakeSubsystem.setMotorFullPowerOut();
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setMotorStopped();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
