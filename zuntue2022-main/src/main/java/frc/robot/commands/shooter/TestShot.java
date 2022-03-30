// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.EastShooter;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class TestShot extends CommandBase {

  /**
   * Creates a new FenderShot.
   */
  private final TowerSubsystem tower;
//  private final ShooterSubsystem shooter;
  private final EastShooter eastShooter;
  private final LimelightSubsystem limelight = LimelightSubsystem.getInstance();

  public TestShot(TowerSubsystem tower, EastShooter eastShooter/*ShooterSubsystem shooter*/) {
    this.tower = tower;
//    this.shooter = shooter;
    this.eastShooter = eastShooter;
    addRequirements(tower, eastShooter /*shooter*/);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tower.setTowerThirdPower();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tower.setTowerMotorsSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
