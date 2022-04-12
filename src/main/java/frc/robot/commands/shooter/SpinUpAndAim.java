// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.*;

public class SpinUpAndAim extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final LimelightSubsystem limelight;
  private final LEDsSubsystem LEDS;

  private int initialBallCount = 0;
  private int ballcount = 0;
  private double towerSpeed = TowerConstants.towerMotorSpeed;
  private double headingError = 0;
  private boolean shotOne = false;

  /**
   * Creates a new Shoot.
   */
  public SpinUpAndAim(ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limelight,
      LEDsSubsystem leds) {

    this.shooterSubsystem = shooterSubsystem;
    this.limelight = limelight;
    this.LEDS = leds;
    addRequirements(shooterSubsystem, limelight, leds);
  }

  @Override
  public void initialize() {
    shooterSubsystem.setShooterRPM(
        limelight.calculateRPM(ShooterConstants.bottomMotorValues),
        limelight.calculateRPM(ShooterConstants.topMotorValues)
    );

    headingError = limelight.getTargetOffsetX();
  }

  @Override
  public void execute() {
    shooterSubsystem.setShooterRPMNotImproved(
        limelight.calculateRPM(ShooterConstants.bottomMotorValues),
        limelight.calculateRPM(ShooterConstants.topMotorValues)
    );
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterToNeutral();
    LEDS.setLEDsDefault();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
