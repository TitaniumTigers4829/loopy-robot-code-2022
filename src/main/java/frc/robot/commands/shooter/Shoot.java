// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class Shoot extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final TowerSubsystem towerSubsystem;
  private final LimelightSubsystem limelight;

  /** Creates a new Shoot. */
  public Shoot(ShooterSubsystem shooterSubsystem, TowerSubsystem towerSubsystem, LimelightSubsystem limelight) {
    this.shooterSubsystem = shooterSubsystem;
    this.towerSubsystem = towerSubsystem;
    this.limelight = limelight;
    addRequirements(shooterSubsystem, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double distance = limelight.calculateDistance();
    double speed = linearInterpolate(distance, ShooterConstants.shootSpeedValues);
    double height = linearInterpolate(distance, ShooterConstants.shootHeightValues);
    shooterSubsystem.setSpeed(speed);
    shooterSubsystem.setHeight(height);
    Timer.delay(1);
    towerSubsystem.setTowerMotorsSpeed(ShooterConstants.towerMotorSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setSpeed(0);
    shooterSubsystem.setHeight(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double linearInterpolate(double distance, double[][] valueTable) {
  
    double lowerXValue = 0;
    double lowerYValue = 0;
    double higherXValue = 0;
    double higherYValue = 0;

    for (int i = 0; i < valueTable.length; i++) {
      if (valueTable[i][0] <= distance && valueTable[i + 1][0] > distance) {
        lowerXValue = valueTable[i][0];
        lowerYValue = valueTable[i][1];
        higherXValue = valueTable[i + 1][0];
        higherYValue = valueTable[i + 1][1];
        break;
      }
    }

    // Gets slope or line connecting points
    double linearSlope = (higherYValue - lowerYValue) / (higherXValue - lowerXValue);

    // Uses point slope form to get the xValue
    return (linearSlope * (distance - lowerXValue) + lowerYValue);

  }
}
