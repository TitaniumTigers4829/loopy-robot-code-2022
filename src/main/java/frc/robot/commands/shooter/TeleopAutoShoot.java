// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import  edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopAutoShoot extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final TowerSubsystem towerSubsystem;
  private final LimelightSubsystem limelight;
  private final DriveSubsystem driveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final DoubleSupplier leftStickY;
  private final DoubleSupplier leftStickX;
  private final DoubleSupplier rightStickY;
  private final BooleanSupplier isFieldRelative;
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

  private int initialBallCount = 0;
  private int ballcount = 0;
  private double towerSpeed = TowerConstants.towerMotorSpeed;
  private double headingError = 0;
//  private boolean shotOne = false;
//  IDK if we will need this, but it could be nice to have...
  private boolean[] arrayOfBalls = new boolean[2];
  private double currentDriveSpeed;
  /**
   * Creates a new Shoot.
   */
  public TeleopAutoShoot(ShooterSubsystem shooterSubsystem,
                         TowerSubsystem towerSubsystem, LimelightSubsystem limelight, DriveSubsystem driveSubsystem,
                         DoubleSupplier leftStickY, DoubleSupplier leftStickX, DoubleSupplier rightStickY, BooleanSupplier isFieldRelative,
                         LEDsSubsystem leds, IntakeSubsystem intakeSubsystem) {

    this.shooterSubsystem = shooterSubsystem;
    this.towerSubsystem = towerSubsystem;
    this.limelight = limelight;
    this.driveSubsystem = driveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.leftStickY = leftStickY;
    this.leftStickX = leftStickX;
    this.rightStickY = rightStickY;
    this.isFieldRelative = isFieldRelative;
    this.LEDS = leds;
    addRequirements(shooterSubsystem, limelight, leds, driveSubsystem);
  }

  @Override
  public void initialize() {
    if (towerSubsystem.getIsBallInBottom()){
      ballcount += 1;
      arrayOfBalls[0] = true;
    }
    if (towerSubsystem.getIsBallInTop()){
      ballcount += 1;
      arrayOfBalls[1] = true;
    }
    initialBallCount = ballcount;
  }

  @Override
  public void execute() {
    updateBallCount();
    if (ballcount < 2) {
      driveSubsystem.drive(-leftStickY.getAsDouble(), -leftStickX.getAsDouble(), -rightStickY.getAsDouble(), isFieldRelative.getAsBoolean());
    } else {
      if (isLimelightInRange()) {
        shooterSubsystem.setShooterRPM(
                limelight.calculateRPM(ShooterConstants.bottomMotorValues),
                limelight.calculateRPM(ShooterConstants.topMotorValues)
        );
        headingError = limelight.getTargetOffsetX();

        // If heading error isn't off by much, it won't move
        if (Math.abs(headingError) < 1) headingError = 0;

        double turnRobotOutput =
                turnProfiledPIDController.calculate(headingError, 0)
                        + turnFeedforward.calculate(turnProfiledPIDController.getSetpoint().velocity);

        driveSubsystem.drive(leftStickY.getAsDouble() * -DriveConstants.kMaxSpeedMetersPerSecond, leftStickX.getAsDouble() * -DriveConstants.kMaxSpeedMetersPerSecond, turnRobotOutput, isFieldRelative.getAsBoolean());
        SmartDashboard.putBoolean("Ready to shoot", isReadyToShoot());
        if (isReadyToShoot()) {
          intakeSubsystem.setMotorCustomPower(0.15);
          LEDS.setLEDsReadyToShoot();
          towerSubsystem.setTowerMotorsSpeed(TowerConstants.towerMotorSpeed);
        } else if (!limelight.hasValidTarget()) {
          LEDS.setLEDsNoValidTarget();
          towerSubsystem.setTowerMotorsSpeed(0);
        } else {
          LEDS.setLEDsShooterLiningUp();
          towerSubsystem.setTowerMotorsSpeed(0);
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterToNeutral();
    towerSubsystem.setTowerOff();
    LEDS.setLEDsDefault();
    intakeSubsystem.setMotorCustomPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }


  // If limelight has low x-offset, has a valid target, and shooter flywheels have low RPM error.
  private boolean isReadyToShoot() {
    return (((Math.abs(headingError) < 3) && (limelight.hasValidTarget()) && shooterSubsystem.isShooterWithinAcceptableError()));
  }

  private boolean isLimelightInRange() {
    return (limelight.hasValidTarget() && (limelight.calculateDistance() < ShooterConstants.maxDistance));
  }

  private void updateBallCount() {
    ballcount = 0;
    if (towerSubsystem.getIsBallInBottom()) {
      ballcount += 1;
      arrayOfBalls[0] = true;
    } else {
      arrayOfBalls[0] = false;
    }
    if (towerSubsystem.getIsBallInTop()){
      ballcount += 1;
      arrayOfBalls[1] = true;
    } else {
      arrayOfBalls[1] = false;
    }
  }
}
