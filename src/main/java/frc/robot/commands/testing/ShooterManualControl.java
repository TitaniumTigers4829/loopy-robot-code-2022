package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class ShooterManualControl extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooterSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterManualControl(ShooterSubsystem subsystem) {
    m_shooterSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
  }

//  @Override
//  public void initialize() {
//  }
//
//  @Override
//  public void execute() {
//  }
//
//  @Override
//  public void end(boolean interrupted) {
//  }

  @Override
  public boolean isFinished() {
    return true;
  }
}