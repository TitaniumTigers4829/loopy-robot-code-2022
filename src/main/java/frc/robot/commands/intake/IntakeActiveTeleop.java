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
   * @param subsystem The climb subsystem used by this command.
   */
  public IntakeActiveTeleop(IntakeSubsystem subsystem) {
    m_intakeSubsystem = subsystem;
    addRequirements(m_intakeSubsystem);  // Use addRequirements() to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    m_intakeSubsystem.setSolenoidDeployed();
//    Timer.delay(0.25);
    m_intakeSubsystem.setMotorFullPowerIn();
  }

  @Override
  public void execute() {
    m_intakeSubsystem.setMotorFullPowerIn();
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setMotorStopped();
//    Timer.delay(0.25);
    m_intakeSubsystem.setSolenoidRetracted();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
