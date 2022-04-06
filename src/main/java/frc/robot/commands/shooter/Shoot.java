// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Shoot extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final TowerSubsystem towerSubsystem;
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

  private int initialBallCount = 0;
  private int ballcount = 0;
  private double towerSpeed = TowerConstants.towerMotorSpeed;
  private double headingError = 0;
  private boolean shotOne = false;

  /**
   * Creates a new Shoot.
   */
  public Shoot(ShooterSubsystem shooterSubsystem,
      TowerSubsystem towerSubsystem, LimelightSubsystem limelight, DriveSubsystem driveSubsystem,
      DoubleSupplier leftStickY, DoubleSupplier leftStickX,
      LEDsSubsystem leds) {

    this.shooterSubsystem = shooterSubsystem;
    this.towerSubsystem = towerSubsystem;
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

    double turnRobotOutput =
        turnProfiledPIDController.calculate(headingError, 0)
            + turnFeedforward.calculate(turnProfiledPIDController.getSetpoint().velocity);

    if (towerSubsystem.getIsBallInBottom()){
      ballcount += 1;
    }
    if (towerSubsystem.getIsBallInTop()){
      ballcount += 1;
    }
    initialBallCount = ballcount;
  }

  @Override
  public void execute() {
    shooterSubsystem.setShooterRPMImproved(
        limelight.calculateRPM(ShooterConstants.bottomMotorValues),
        limelight.calculateRPM(ShooterConstants.topMotorValues)
    );


//    old:

//    if ((towerSubsystem.getIsBallInBottom()) && (towerSubsystem.getIsBallInTop())) {
//      ballcount = 2;
//    } else if ((towerSubsystem.getIsBallInTop()) && !(towerSubsystem.getIsBallInBottom())) {
//      ballcount = 1;
//    } else if ((towerSubsystem.getIsBallInBottom()) && !(towerSubsystem.getIsBallInTop())) {
//      ballcount = 1;
//    } else {
//      ballcount = 0;
//    }
    if ((initialBallCount == 2) && !(towerSubsystem.getIsBallInBottom())) {
      ballcount = 1;
    }
    if (initialBallCount == 1) {
      shotOne = true;
    } else if ((initialBallCount == 2) && (ballcount == 1)) {
      shotOne = true;
    }

//    towerSpeed = (shotOne ? 0.34 : TowerConstants.towerMotorSpeed);

    headingError = limelight.getTargetOffsetX();

    // If heading error isn't off by much, it won't move
    if (Math.abs(headingError) < 1) headingError = 0;

    double turnRobotOutput =
        turnProfiledPIDController.calculate(headingError, 0)
            + turnFeedforward.calculate(turnProfiledPIDController.getSetpoint().velocity);

    driveSubsystem.drive(leftStickY.getAsDouble() * -DriveConstants.kMaxSpeedMetersPerSecond, leftStickX.getAsDouble() * -DriveConstants.kMaxSpeedMetersPerSecond, turnRobotOutput, true);

    SmartDashboard.putBoolean("Ready to shoot", isReadyToShoot());
    if (isReadyToShoot()) {
      LEDS.setLEDsReadyToShoot();
      towerSubsystem.setTowerMotorsSpeed(TowerConstants.towerMotorSpeed);
    } else {
      LEDS.setLEDsShooterLiningUp();
      towerSubsystem.setTowerMotorsSpeed(0);
    }
//    SmartDashboard.putNumber("Target offset X: ", limelight.getTargetOffsetX());
//    SmartDashboard.putBoolean("Has valid target: ", limelight.hasValidTarget());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterToNeutral();
    towerSubsystem.setTowerOff();
    LEDS.setLEDsDefault();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean isReadyToShoot() {
    // If low offset, has a valid target, and shooter flywheels are spun up.
    // FIXME: what is going on with the 1.1 stuff in the shooterSubsystem?  Is that permanent?
    // FIXME: Can we re tune the PID loop now that we have better CAN utilization? (It kinda gets it right now, but it should be better.)
    // We want to never miss any shots.
    SmartDashboard.putBoolean("rpm within range: ", shooterSubsystem.isShooterWithinAcceptableError());
    SmartDashboard.putNumber("limelight offset: ", Math.abs(limelight.getTargetOffsetX()));
    return (((Math.abs(headingError) < 5) && (limelight.hasValidTarget()) && (shooterSubsystem.isShooterWithinAcceptableError())));
  }
}
