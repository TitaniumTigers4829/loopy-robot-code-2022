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
    double speed = limelight.calculateSpeed();
    shooterSubsystem.setSpeed(speed);
    Timer.delay(1);
    towerSubsystem.setTowerMotorsSpeed(ShooterConstants.towerMotorSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
