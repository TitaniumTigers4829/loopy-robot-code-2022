// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Balance extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final DoubleSupplier leftStickY;
  private final DoubleSupplier leftStickX;
  private final DoubleSupplier rightStickX;
  
  private final PIDController balancePIDController = new PIDController(.003, 0, 0);

  /** Creates a new Balance. */
  public Balance(DriveSubsystem driveSubsystem, DoubleSupplier leftStickY, DoubleSupplier leftStickX, DoubleSupplier rightStickX) {
    this.driveSubsystem = driveSubsystem;
    this.leftStickY = leftStickY;
    this.leftStickX = leftStickX;
    this.rightStickX = rightStickX;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double balanceError = driveSubsystem.getPitch();
    if (balanceError < 2 && balanceError > -2) {
      balanceError = 0;
    }
    
    double driveOutput = balancePIDController.calculate(balanceError, 0);

    if (leftStickY.getAsDouble() + driveOutput >= 1) {
      driveOutput = 1;
    } else if (leftStickY.getAsDouble() + driveOutput <= -1) {
      driveOutput = -1;
    } else {
      driveOutput += leftStickY.getAsDouble();
    }

    driveSubsystem.drive(driveOutput * -DriveConstants.kMaxSpeedMetersPerSecond,
      leftStickX.getAsDouble() * -DriveConstants.kMaxSpeedMetersPerSecond, rightStickX.getAsDouble() * -DriveConstants.kMaxRotationalSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
