// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SetDriveSpeed extends CommandBase {

  private DriveSubsystem driveSubsystem;
  private final double xSpeed;
  private final double ySpeed;

  /** Creates a new SwerveDriveCommand. */
  public SetDriveSpeed(DriveSubsystem driveSubsystem, double xSpeed, double ySpeed) {
    this.driveSubsystem = driveSubsystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.zeroHeading();
    driveSubsystem.drive(xSpeed, ySpeed, 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.drive(xSpeed, ySpeed, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(xSpeed, ySpeed, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
