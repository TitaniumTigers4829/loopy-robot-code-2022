// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;

  private final CANCoder m_turnEncoder;
  // Driving encoder uses the integrated FX encoder
  // e.g. testMotor.getSelectedSensorPosition();


  // Redundant with code below, but could be good starting spot.
  // Gains are for example purposes only - must be determined for your own robot!
  // private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  // PID controller for velocity. DO NOT SET kD.  It is redundant as setVoltage() already controls this
  private final PIDController m_drivePIDController =
      new PIDController(
          ModuleConstants.kPModuleDriveController,
          ModuleConstants.kIModuleDriveController, // 0
          ModuleConstants.kDModuleDriveController // 0
      );

  ShuffleboardTab PIDtab = Shuffleboard.getTab("PID Tuning");


  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turnPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurnController,
          ModuleConstants.kIModuleTurnController, // 0
          ModuleConstants.kDModuleTurnController,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  // In the pose example.
  // NOTE: The passed-in gains must have units consistent with the distance units, or a compile-time error will be thrown.
  // kS should have units of volts, kV should have units of volts * seconds / distance, and kA should have units of volts * seconds^2 / distance.
//  // Gains are for example purposes only - must be determined for your own robot!
//  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
//  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
      DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter);

  SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(
      DriveConstants.ksTurning, DriveConstants.kvTurning);

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double angleZero,
      ShuffleboardLayout container
      ) {

    // Initialize the motors
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turningMotor = new WPI_TalonFX(turningMotorChannel);


    // Configure the encoders for both motors

    m_driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    this.m_turnEncoder = new CANCoder(turningEncoderChannel);
    // Pretty sure this is to handle all the modules starting in the same position/orientation.
    this.m_turnEncoder.configMagnetOffset(-1 * angleZero);
    this.m_turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);


    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }


  public double getModuleHeading(){
    return this.m_turnEncoder.getAbsolutePosition();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    double m_speedMetersPerSecond =
        ModuleConstants.kDrivetoMetersPerSecond * m_driveMotor.getSelectedSensorVelocity();

    double m_turningRadians =
        ((2*Math.PI)/360) * m_turnEncoder.getAbsolutePosition();

    return new SwerveModuleState(m_speedMetersPerSecond, new Rotation2d(m_turningRadians));
  }


  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    double m_speedMetersPerSecond =
        ModuleConstants.kDrivetoMetersPerSecond * m_driveMotor.getSelectedSensorVelocity();

    SmartDashboard.putNumber("m_speedMetersPerSecond", m_speedMetersPerSecond);

    double m_turnRadians =
        ((2*Math.PI)/360) * m_turnEncoder.getAbsolutePosition();

    SmartDashboard.putNumber("m_turnRadians", m_turnRadians);
    SmartDashboard.putNumber("Turning Degrees", m_turnEncoder.getAbsolutePosition());


    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turnRadians));


    SmartDashboard.putNumber("desiredState", desiredState.speedMetersPerSecond);

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_speedMetersPerSecond, state.speedMetersPerSecond)
            + driveFeedforward.calculate(state.speedMetersPerSecond);;

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput =
        m_turnPIDController.calculate(m_turnRadians, state.angle.getRadians())
            + turnFeedForward.calculate(m_turnPIDController.getSetpoint().velocity);

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);

    SmartDashboard.putNumber("turnPID Setpoint Velocity", m_turnPIDController.getSetpoint().velocity);
    SmartDashboard.putNumber("PID driveOutput", driveOutput);
    SmartDashboard.putNumber("PID turnOutput", turnOutput);
    SmartDashboard.putNumber("Feedforward", driveFeedforward.calculate(desiredState.speedMetersPerSecond));
    SmartDashboard.putNumber("PID Output", m_drivePIDController.calculate(m_speedMetersPerSecond, state.speedMetersPerSecond));
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    m_turnEncoder.setPosition(0);
    m_driveMotor.setSelectedSensorPosition(0);
  }
}