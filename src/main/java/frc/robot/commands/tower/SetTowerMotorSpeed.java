// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tower;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TowerSubsystem;

public class SetTowerMotorSpeed extends CommandBase {

  private TowerSubsystem towerSubsystem;
  private final double speed;

  /** Creates a new SetTowerMotorSpeed. */
  public SetTowerMotorSpeed(TowerSubsystem towerSubsystem, Double speed) {
    this.towerSubsystem = towerSubsystem;
    this.speed = speed;
    addRequirements(towerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    towerSubsystem.setTowerMotorsSpeed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
