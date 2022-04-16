// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tower;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class SetTowerMotorSpeed extends CommandBase {
  
  private TowerSubsystem towerSubsystem;
  private ShooterSubsystem shooterSubsystem;
//  private LEDsSubsystem leds;
  private final double speed;
  
  /** Creates a new SetTowerMotorSpeed. */
  public SetTowerMotorSpeed(TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem, double speed) {
    this.towerSubsystem = towerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
//    this.leds = leds;
    this.speed = speed;
    addRequirements(towerSubsystem, shooterSubsystem);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    towerSubsystem.setTowerMotorsSpeed(speed);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    towerSubsystem.setTowerMotorsSpeed(speed);
//    towerSubsystem.setBottomMotorOutputManual(speed);
//    shooterSubsystem.setSpeed1(-0.4, -0.4);
//    leds.setLEDsBackward();
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    towerSubsystem.setTowerMotorsSpeed(0);
    shooterSubsystem.setSpeed1(0, 0);
//    leds.setLEDsDefault();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
