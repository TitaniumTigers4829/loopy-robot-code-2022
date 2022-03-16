package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.BooleanSupplier;

public class IntakeActiveTeleop extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intakeSubsystem;

  private final BooleanSupplier m_active;

  /**
   * Creates IntakeActiveTeleop. By default, a "while held" command.
   *
   * @param subsystem The climb subsystem used by this command.
   * @param active    Effectively makes command work like a "while held" command.
   *                  <p>
   *                  I feel like this is ok to make this the only behavior, especially with G204,
   *                  we do not want the intake down if not necessary.
   */
  public IntakeActiveTeleop(IntakeSubsystem subsystem, BooleanSupplier active) {
    m_intakeSubsystem = subsystem;
    addRequirements(m_intakeSubsystem);  // Use addRequirements() to declare subsystem dependencies.

    m_active = active;
  }

  @Override
  public void initialize() {
    m_intakeSubsystem.setSolenoidDeployed();
    // Timer.delay(0.5); // Slight delay
//    m_intakeSubsystem.setMotorFullPowerIn();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
//    m_intakeSubsystem.setMotorStopped();
    // Timer.delay(0.5); // Slight delay
    m_intakeSubsystem.setSolenoidRetracted();
  }

  @Override
  public boolean isFinished() {
    return m_active.getAsBoolean();
  }
}
