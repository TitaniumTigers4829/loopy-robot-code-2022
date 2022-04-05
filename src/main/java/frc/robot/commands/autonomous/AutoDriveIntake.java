// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.IntakeWithTower;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TowerSubsystem;

// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveIntake extends ParallelCommandGroup {

  /**
   * This command has the robot back up while intaking
   */
  public AutoDriveIntake(DriveSubsystem drive, IntakeSubsystem intake, TowerSubsystem tower, double backupSpeed) {

    addCommands(
        new IntakeWithTower(intake, tower),
        new SetDriveSpeed(drive, backupSpeed, 0)
    );
  }
}
