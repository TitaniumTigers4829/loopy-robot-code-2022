// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.WaitCommand;
import frc.robot.commands.tower.SetTowerMotorSpeed;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class FenderShot extends SequentialCommandGroup {

  /** Creates a new FenderShot. */
  public FenderShot(ShooterSubsystem shooterSubsystem, TowerSubsystem towerSubsystem) {
    addCommands(
      new SetShooterHeight(shooterSubsystem, ShooterConstants.fenderShotHeight),
      new SetFlyWheelSpeed(shooterSubsystem, ShooterConstants.fenderShotSpeed),
      // Waits a second for flywheels to get up to speed
//      new WaitCommand(1),
//      new RunCommand(()-> Timer.delay(1)),
      // Shoots the ball
      // TODO: Make it so the tower motors go a certain distance so we can control if there are multiple balls loaded
      new SetTowerMotorSpeed(towerSubsystem, 0.34),
//      new RunCommand(()-> Timer.delay(0.5)),
      // Stops the flywheels, the tower motors, and sets the shooter height to 50%
      new ResetShooter(shooterSubsystem),
      new SetTowerMotorSpeed(towerSubsystem, 0.0)
    );
  }

}
