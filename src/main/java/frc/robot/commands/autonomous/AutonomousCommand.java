// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.FenderShot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class AutonomousCommand extends SequentialCommandGroup {
  /** Add your docs here. */
  public AutonomousCommand(ShooterSubsystem shooterSubsystem, TowerSubsystem towerSubsystem, DriveSubsystem driveSubsystem) {
    addCommands(
      // Starts assuming that we the robot is in the correct position for a fender shot
      new FenderShot(shooterSubsystem, towerSubsystem)
      // Moves backwards
      //driveSubsystem.drive(-.25, 0, 0, false)
    );
  }
}