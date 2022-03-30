package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class ShooterManualControl extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooterSubsystem;
  private final double m_speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param speed
   */
  public ShooterManualControl(ShooterSubsystem subsystem, double speed) {
    m_shooterSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);

    m_speed = speed;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_shooterSubsystem.setSpeed(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}