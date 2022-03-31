// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class Shoot extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final TowerSubsystem towerSubsystem;
  private final LimelightSubsystem limelight;
  private final DriveSubsystem driveSubsystem;
  private final DoubleSupplier leftStickY;
  private final DoubleSupplier leftStickX;
  private final BooleanSupplier rightBumper;
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

  /**
   * Creates a new Shoot.
   */
  public Shoot(ShooterSubsystem shooterSubsystem,
      TowerSubsystem towerSubsystem, LimelightSubsystem limelight, DriveSubsystem driveSubsystem,
      DoubleSupplier leftStickY, DoubleSupplier leftStickX, JoystickButton rightBumper,
      LEDsSubsystem leds) {

    this.shooterSubsystem = shooterSubsystem;
    this.towerSubsystem = towerSubsystem;
    this.limelight = limelight;
    this.driveSubsystem = driveSubsystem;
    this.leftStickY = leftStickY;
    this.leftStickX = leftStickX;
    this.rightBumper = rightBumper;
    this.LEDS = leds;
    addRequirements(shooterSubsystem, limelight, leds);
  }

  @Override
  public void initialize() {
    shooterSubsystem.setShooterRPM(
        limelight.calculateRPM(ShooterConstants.bottomMotorValues),
        limelight.calculateRPM(ShooterConstants.topMotorValues)
    );

    double headingError = limelight.getTargetOffsetX();

    double turnRobotOutput =
        turnProfiledPIDController.calculate(headingError, 0)
            + turnFeedforward.calculate(turnProfiledPIDController.getSetpoint().velocity);

    driveSubsystem.drive(leftStickY.getAsDouble(), leftStickX.getAsDouble(), turnRobotOutput, true);

  }

  @Override
  public void execute() {
    shooterSubsystem.setShooterRPM(
        limelight.calculateRPM(ShooterConstants.bottomMotorValues),
        limelight.calculateRPM(ShooterConstants.topMotorValues)
    );

    double headingError = limelight.getTargetOffsetX();

    // If heading error isn't off by much, it won't move
    if (Math.abs(headingError) < 1) headingError = 0;

    double turnRobotOutput =
        turnProfiledPIDController.calculate(headingError, 0)
            + turnFeedforward.calculate(turnProfiledPIDController.getSetpoint().velocity);

    driveSubsystem.drive(-1 * leftStickY.getAsDouble(), -1 * leftStickX.getAsDouble(), turnRobotOutput, true);

    if ((Math.abs(headingError) < 3) && (limelight.hasValidTarget())) {
      towerSubsystem.setTowerMotorsSpeed(TowerConstants.towerMotorSpeed);
    } else {
      towerSubsystem.setTowerMotorsSpeed(0);
    }

    if (headingError < 3 && shooterSubsystem.getShooterAverageRPMError() < 200) {
      LEDS.setLEDsReadyToShoot();
    } else {
      LEDS.setLEDsShooterLiningUp();
    }

//    SmartDashboard.putNumber("Target offset X: ", limelight.getTargetOffsetX());
//    SmartDashboard.putBoolean("Has valid target: ", limelight.hasValidTarget());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterRPM(0, 0);
    driveSubsystem.getDefaultCommand().schedule();
    towerSubsystem.setTowerOff();
    LEDS.setLEDsDefault();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
