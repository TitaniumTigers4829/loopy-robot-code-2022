package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPIDtesting extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;

  public ShooterPIDtesting (ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
  }

  public void execute() {
    shooterSubsystem.setShooterRPM(
        1000,
        1000
    );
  }
}
