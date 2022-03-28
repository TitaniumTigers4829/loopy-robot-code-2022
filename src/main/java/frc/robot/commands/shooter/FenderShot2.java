// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class FenderShot2 extends CommandBase {

  /**
   * Creates a new FenderShot2.
   */
  private final TowerSubsystem tower;
  private final ShooterSubsystem shooter;
  private final boolean isAuto;
  private boolean shotTop = false;
  private int ballCount = 0;
  private int iteration = 0;
  private final boolean isFancyShooting = false;

  public FenderShot2(TowerSubsystem tower, ShooterSubsystem shooter, boolean isAuto) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tower = tower;
    this.shooter = shooter;
    this.isAuto = isAuto;
    addRequirements(tower, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // This decides what type of shot it will be
    if (isAuto) {
      shooter.setSpeed(ShooterConstants.fenderShotSpeed);
    } else {
      shooter.setSpeed(ShooterConstants.tarmacShotSpeed);
    }
    Timer.delay(1);
    if (tower.getIsBallInBottom()) {
      ballCount++;
    }
    if (tower.getIsBallInTop()) {
      ballCount++;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((isFancyShooting) && (!isAuto)) {
      // Adds a little delay in between the shooting of two balls
      if ((ballCount == 2) && (!shotTop)) {
        if (tower.getIsBallInTop()) {
          // Shoots the top ball if there are two balls
          tower.setTopMotorOutputManual(ShooterConstants.towerMotorSpeed);
        } else {
          tower.setTowerMotorsSpeed(0);
          shotTop = true;
        }
      }

      if ((ballCount == 2) && (shotTop)) {
        if (iteration <= 25) {
          // Execute is called 50 times a second, this adds a half second delay to the firing of the second ball
          iteration++;
        } else {
          // Shoots the bottom ball if there were two balls when originally fired
          tower.setTowerMotorsSpeed(ShooterConstants.towerMotorSpeed);
        }
      }

      if (ballCount == 1) {
        // Shoots the top ball if there is only one ball
        tower.setTowerMotorsSpeed(ShooterConstants.towerMotorSpeed);
      }
    } else {
      tower.setTowerMotorsSpeed(ShooterConstants.towerMotorSpeed);
    }

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
