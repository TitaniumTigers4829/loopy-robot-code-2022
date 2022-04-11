// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.commands.tower.TowerIntake;
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
  private int overshoot_elimination_counter = 0;

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
  int iteration = 0;

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
    shooterSubsystem.setShooterRPMImproved(
        limelight.calculateRPM(ShooterConstants.bottomMotorValues),
        limelight.calculateRPM(ShooterConstants.topMotorValues)
    );

    headingError = limelight.getTargetOffsetX();

//    double turnRobotOutput =
//        turnProfiledPIDController.calculate(headingError, 0)
//            + turnFeedforward.calculate(turnProfiledPIDController.getSetpoint().velocity);

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
//    if ((initialBallCount == 2) && !(towerSubsystem.getIsBallInBottom())) {
//      ballcount = 1;
//    }
//    if (initialBallCount == 1) {
//      shotOne = true;
//    } else if ((initialBallCount == 2) && (ballcount == 1)) {
//      shotOne = true;
//    }

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
//      towerSubsystem.setTopMotorOutputManual(TowerConstants.towerMotorSpeed);
//      Timer.delay(0.25);
//      towerSubsystem.setBottomMotorOutputManual(TowerConstants.towerMotorSpeed);

//      if (towerSubsystem.getIsBallInTop()){
//        Timer.delay(0.2);
//        towerSubsystem.setTopMotorOutputManual(TowerConstants.towerMotorSpeed);
//      }
//      towerSubsystem.setBottomMotorOutputManual(TowerConstants.towerMotorSpeed - 0.1);
    } else {
      LEDS.setLEDsShooterLiningUp();
      towerSubsystem.setTowerMotorsSpeed(0);
    }
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


  // If limelight has low x-offset, has a valid target, and shooter flywheels have low RPM error.
  private boolean isReadyToShoot() {
//    if (shooterSubsystem.isShooterWithinAcceptableError()) {
//      overshoot_elimination_counter += 1;
//    } else {
//      overshoot_elimination_counter = 0;
//    }
//    SmartDashboard.putBoolean("overshoot_counter: ", overshoot_elimination_counter > 2);
//    SmartDashboard.putNumber("limelight offset: ", Math.abs(limelight.getTargetOffsetX()));
//    return shooterSubsystem.isShooterWithinAcceptableError();
    return (((Math.abs(headingError) < 3) && (limelight.hasValidTarget()) && shooterSubsystem.isShooterWithinAcceptableError()));
  }
}
