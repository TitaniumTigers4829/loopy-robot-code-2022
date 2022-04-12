// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.*;


public class AutoRevShoot extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final LimelightSubsystem limelight;

  public AutoRevShoot(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelight) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelight = limelight;
    addRequirements(shooterSubsystem, limelight);
  }

  @Override
  public void initialize() {
    shooterSubsystem.setShooterRPM(
        limelight.calculateRPM(ShooterConstants.bottomMotorValues),
        limelight.calculateRPM(ShooterConstants.topMotorValues)
    );
  }

  @Override
  public void execute() {
    shooterSubsystem.setShooterRPMNotImproved(
        limelight.calculateRPM(ShooterConstants.bottomMotorValues),
        limelight.calculateRPM(ShooterConstants.topMotorValues)
    );
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterToNeutral();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
