// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class RevAndAim extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final LimelightSubsystem limelight;
  private final DriveSubsystem driveSubsystem;
  private final DoubleSupplier leftStickY;
  private final DoubleSupplier leftStickX;
  private final LEDsSubsystem LEDS;

  private final ProfiledPIDController turnProfiledPIDController = new ProfiledPIDController(
      ShooterConstants.turnkP,
      ShooterConstants.turnkI,
      ShooterConstants.turnkD,
      new TrapezoidProfile.Constraints(
          ShooterConstants.kMaxTurnAngularSpeedRadiansPerSecond,
          ShooterConstants.kMaxTurnAngularAccelerationRadiansPerSecondSquared)
  );
  private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(
      ShooterConstants.ksTurning, ShooterConstants.kvTurning
  );
  private double headingError = 0;

  /**
   * Creates a new Shoot.
   */
  public RevAndAim(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelight,
      DriveSubsystem driveSubsystem,
      DoubleSupplier leftStickY, DoubleSupplier leftStickX,
      LEDsSubsystem leds) {

    this.shooterSubsystem = shooterSubsystem;
    this.limelight = limelight;
    this.driveSubsystem = driveSubsystem;
    this.leftStickY = leftStickY;
    this.leftStickX = leftStickX;
    this.LEDS = leds;
    addRequirements(shooterSubsystem, limelight, leds, driveSubsystem);
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

    headingError = limelight.getTargetOffsetX();

    // If heading error isn't off by much, it won't move
    if (Math.abs(headingError) < 1) {
      headingError = 0;
    }

    double turnRobotOutput =
        turnProfiledPIDController.calculate(headingError, 0)
            + turnFeedforward.calculate(turnProfiledPIDController.getSetpoint().velocity);

    driveSubsystem.drive(leftStickY.getAsDouble() * -DriveConstants.kMaxSpeedMetersPerSecond,
        leftStickX.getAsDouble() * -DriveConstants.kMaxSpeedMetersPerSecond, turnRobotOutput, true);

    if (!limelight.hasValidTarget()) {
      LEDS.setLEDsShooterLiningUp();
    } else {
      LEDS.setLEDsAButton();
    }

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


  private boolean isReadyToShoot() {
    return (((Math.abs(headingError) < 3) && (limelight.hasValidTarget())
        && shooterSubsystem.isShooterWithinAcceptableError()));
  }
}