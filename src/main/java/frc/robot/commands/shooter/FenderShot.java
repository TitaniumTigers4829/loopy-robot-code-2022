// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class FenderShot extends SequentialCommandGroup {

  /** Creates a new FenderShot. */
  public FenderShot(ShooterSubsystem shooterSubsystem) {
    addCommands(
      new SetShooterHeight(shooterSubsystem, ShooterConstants.fenderShotHeight),
      // This uses a timer for delay
      new SetFlyWheelSpeed(shooterSubsystem, ShooterConstants.fenderShotSpeed),
      // After the timer is done, it does this to shoot the ball
    );
  }

}
