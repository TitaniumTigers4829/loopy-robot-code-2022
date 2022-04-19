// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.deprecated;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.AutoDriveIntake;
import frc.robot.commands.autonomous.SetDriveSpeed;
import frc.robot.commands.autonomous.TwoBallAutoShoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class OldTwoBallAutoCommand extends SequentialCommandGroup {

  /**
   * Add your docs here.
   */
  public OldTwoBallAutoCommand(ShooterSubsystem shooterSubsystem, TowerSubsystem towerSubsystem,
      DriveSubsystem driveSubsystem, LEDsSubsystem ledsSubsystem, IntakeSubsystem intake) {
    addCommands(
        // Moves backwards
        new AutoDriveIntake(driveSubsystem, intake, towerSubsystem, 1.2).withTimeout(2.25),
        // Stops the robot
        new SetDriveSpeed(driveSubsystem, 0, 0).withTimeout(0.5),
        // drive forward a bit
        new SetDriveSpeed(driveSubsystem, -1, 0).withTimeout(1.5),
        // Stops the robot
        new SetDriveSpeed(driveSubsystem, 0, 0).withTimeout(0.5),
        // shoots
        new TwoBallAutoShoot(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
            driveSubsystem, ledsSubsystem).withTimeout(5)
    );
  }
}
