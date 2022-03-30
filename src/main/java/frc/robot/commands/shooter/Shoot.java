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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EastShooter;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import java.awt.color.ProfileDataException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Shoot extends CommandBase {

  //  private final ShooterSubsystem shooterSubsystem;
  private final EastShooter eastShooter;
  private final TowerSubsystem towerSubsystem;
  private final LimelightSubsystem limelight;
  private final DriveSubsystem driveSubsystem;
  private final DoubleSupplier leftStickY;
  private final DoubleSupplier leftStickX;
  private final BooleanSupplier rightBumper;
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
  public Shoot(/*ShooterSubsystem shooterSubsystem*/EastShooter eastShooter,
      TowerSubsystem towerSubsystem, LimelightSubsystem limelight, DriveSubsystem driveSubsystem,
      DoubleSupplier leftStickY, DoubleSupplier leftStickX, JoystickButton rightBumper) {

//    this.shooterSubsystem = shooterSubsystem;
    this.eastShooter = eastShooter;
    this.towerSubsystem = towerSubsystem;
    this.limelight = limelight;
    this.driveSubsystem = driveSubsystem;
    this.leftStickY = leftStickY;
    this.leftStickX = leftStickX;
    this.rightBumper = rightBumper;
    addRequirements(/*shooterSubsystem*/eastShooter, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
//    shooterSubsystem.setSpeed(limelight.calculateSpeed());
    eastShooter.setShooterRPM(
        limelight.calculateRPM(ShooterConstants.bottomMotorValues),
        limelight.calculateRPM(ShooterConstants.topMotorValues)
    );

    double headingError = limelight.getTargetOffsetX();

    double turnRobotOutput =
        turnProfiledPIDController.calculate(headingError, 0)
        + turnFeedforward.calculate(turnProfiledPIDController.getSetpoint().velocity);

    driveSubsystem.drive(leftStickY.getAsDouble(), leftStickX.getAsDouble(), turnRobotOutput, true);

    Timer.delay(1);
    towerSubsystem.setTowerMotorsSpeed(TowerConstants.towerMotorSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    eastShooter.setShooterRPM(
        limelight.calculateRPM(ShooterConstants.bottomMotorValues),
        limelight.calculateRPM(ShooterConstants.topMotorValues)
    );

    SmartDashboard.putNumber("calculated bottom rpm: ", limelight.calculateRPM(ShooterConstants.bottomMotorValues));

    double headingError = limelight.getTargetOffsetX();

    double turnRobotOutput =
        turnProfiledPIDController.calculate(headingError, 0)
            + turnFeedforward.calculate(turnProfiledPIDController.getSetpoint().velocity);

    driveSubsystem.drive(leftStickY.getAsDouble(), leftStickX.getAsDouble(), turnRobotOutput, true);

    SmartDashboard.putNumber("turnRobotOutput: ", turnRobotOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
//    shooterSubsystem.setSpeed(0);
    eastShooter.setShooterRPM(0, 0);
    driveSubsystem.getDefaultCommand().schedule();
    towerSubsystem.setTowerOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
