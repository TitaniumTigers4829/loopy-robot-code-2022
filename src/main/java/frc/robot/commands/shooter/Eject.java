// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class Eject extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final TowerSubsystem towerSubsystem;

  /**
   * Creates a new Shoot.
   */
  public Eject(ShooterSubsystem shooterSubsystem,
      TowerSubsystem towerSubsystem) {

    this.shooterSubsystem = shooterSubsystem;
    this.towerSubsystem = towerSubsystem;
    addRequirements(shooterSubsystem, towerSubsystem);
  }

  @Override
  public void initialize() {
//    shooterSubsystem.setShooterRPM(
//        ShooterConstants.kBottomEjectRPM,
//        ShooterConstants.kTopEjectRPM
//    );
    shooterSubsystem.setSpeed1(
        0.3,
        0.3
    );
    Timer.delay(1);
    towerSubsystem.setTowerMotorsSpeed(TowerConstants.towerMotorSpeed);
  }

  @Override
  public void execute() {
    shooterSubsystem.setSpeed1(
        0.3,
        0.3
    );
//    if (Math.abs(shooterSubsystem.getShooterAverageRPMError()) <= 150){
//    }

//    SmartDashboard.putNumber("Target offset X: ", limelight.getTargetOffsetX());
//    SmartDashboard.putBoolean("Has valid target: ", limelight.hasValidTarget());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setSpeed1(0, 0);
    towerSubsystem.setTowerOff();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
