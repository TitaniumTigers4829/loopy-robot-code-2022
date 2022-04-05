// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
    shooterSubsystem.setShooterRPM(
        ShooterConstants.kBottomEjectRPM,
        ShooterConstants.kTopEjectRPM
    );
  }
  @Override
  public void execute() {
    shooterSubsystem.setShooterRPM(
        ShooterConstants.kBottomEjectRPM,
        ShooterConstants.kTopEjectRPM
    );
//    if (Math.abs(shooterSubsystem.getShooterAverageRPMError()) <= 150){
      towerSubsystem.setTowerMotorsSpeed(TowerConstants.towerMotorSpeed);
//    }

//    SmartDashboard.putNumber("Target offset X: ", limelight.getTargetOffsetX());
//    SmartDashboard.putBoolean("Has valid target: ", limelight.hasValidTarget());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterRPM(0, 0);
    towerSubsystem.setTowerOff();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
