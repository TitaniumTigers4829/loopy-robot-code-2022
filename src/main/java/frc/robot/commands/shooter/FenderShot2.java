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
  /** Creates a new FenderShot2. */
  private TowerSubsystem tower;
  private ShooterSubsystem shooter;
  private boolean isAuto;
  private boolean done = false;
  public FenderShot2(TowerSubsystem tower, ShooterSubsystem shooter, boolean isAuto) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tower = tower;
    this.shooter = shooter;
    this.isAuto = isAuto;
    addRequirements(tower, shooter);
  }
  int ballcount = 0;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!isAuto) {
      shooter.setHeight(ShooterConstants.fenderShotHeight);
      shooter.setSpeed(ShooterConstants.fenderShotSpeed);
    } else {
      shooter.setHeight(0.3);
      shooter.setSpeed(0.57);
    }
    Timer.delay(1);
    if (tower.getIsBallInBottom()) {
      ballcount ++;
    }
    if (tower.getIsBallInTop()) {
      ballcount ++;
    }
  }
  private boolean shotBottom = false;
  private boolean shotTop = false;
  private int iteration = 0;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
//    if (!tower.getIsBallInBottom() && !tower.getIsBallInTop()){
//      done = true;
//    }
//    else{
      tower.setTowerMotorsSpeed(0.34);
//    }
//    if ((ballcount == 2) && (!shotTop)){
//      if (tower.getIsBallInTop()) {
//        tower.setTowerMotorsSpeed(0.34);
//      } else {
//        tower.setTowerMotorsSpeed(0);
//        shotTop = true;
//      }
//    }
//    if ((ballcount == 2) && (shotTop)) {
//      if (iteration <= 25){
//        iteration ++;
//      } else {
//        tower.setTowerMotorsSpeed(0.34);
//      }
//    }
//    if (ballcount == 1){
//      tower.setTowerMotorsSpeed(0.34);
//    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
//    shooter.setHeight(0);
    shooter.setSpeed(0);
    tower.setTowerMotorsSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
