// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class LowShot extends CommandBase {

  /**
   * Creates a new FenderShot.
   */
  private final TowerSubsystem tower;
  private final ShooterSubsystem shooter;

  public LowShot(TowerSubsystem tower, ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tower = tower;
    this.shooter = shooter;
    addRequirements(tower, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
//    shooter.setHeight(ShooterConstants.lowShotHeight);
    shooter.setSpeed(ShooterConstants.lowShotSpeed);
    Timer.delay(1);
    tower.setTopMotorOutputManual(ShooterConstants.towerMotorSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
//    shooter.setHeight(ShooterConstants.lowShotHeight);
    shooter.setSpeed(ShooterConstants.lowShotSpeed);
    tower.setTopMotorOutputManual(TowerConstants.towerMotorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setSpeed(0);
    tower.setTowerMotorsSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
