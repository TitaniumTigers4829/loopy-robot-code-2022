// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.*;


public class AutoShootIntake extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final TowerSubsystem towerSubsystem;
  private final LimelightSubsystem limelight;
  private final DriveSubsystem driveSubsystem;
  private final LEDsSubsystem LEDS;
  private final IntakeSubsystem intakeSubsystem;

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
   .
   */
  public AutoShootIntake(ShooterSubsystem shooterSubsystem,
      TowerSubsystem towerSubsystem, LimelightSubsystem limelight, DriveSubsystem driveSubsystem,
      LEDsSubsystem leds, IntakeSubsystem intakeSubsystem) {

    this.shooterSubsystem = shooterSubsystem;
    this.towerSubsystem = towerSubsystem;
    this.limelight = limelight;
    this.driveSubsystem = driveSubsystem;
    this.LEDS = leds;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(shooterSubsystem, towerSubsystem, limelight, driveSubsystem, leds, intakeSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.setShooterRPM(
        limelight.calculateRPM(ShooterConstants.bottomMotorValues),
        limelight.calculateRPM(ShooterConstants.topMotorValues)
    );
  }

  @Override
  public void execute() {
    shooterSubsystem.setShooterRPM(
        limelight.calculateRPM(ShooterConstants.bottomMotorValues),
        limelight.calculateRPM(ShooterConstants.topMotorValues)
    );

    if (!towerSubsystem.getIsBallInTop()) {
      intakeSubsystem.setSolenoidDeployed();
      intakeSubsystem.setMotorFullPowerIn();
    } else {
      intakeSubsystem.setSolenoidRetracted();
      intakeSubsystem.setMotorStopped();
    }
    headingError = limelight.getTargetOffsetX();

    // If heading error isn't off by much, it won't move
//    if (Math.abs(headingError) < 1) headingError = 0;

    double turnRobotOutput =
        turnProfiledPIDController.calculate(headingError, 0)
            + turnFeedforward.calculate(turnProfiledPIDController.getSetpoint().velocity);

    driveSubsystem.drive(0, 0, turnRobotOutput, true);

    SmartDashboard.putBoolean("Ready to shoot", isReadyToShoot());
    if (isReadyToShoot()) {
      LEDS.setLEDsReadyToShoot();
      towerSubsystem.setTowerMotorsSpeed(TowerConstants.towerMotorSpeed);
    } else if (!limelight.hasValidTarget()) {
      LEDS.setLEDsNoValidTarget();
      towerSubsystem.setTowerMotorsSpeed(0);
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
