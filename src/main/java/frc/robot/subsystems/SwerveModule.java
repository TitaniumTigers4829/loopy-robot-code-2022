// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import java.util.Objects;

public class SwerveModule {
  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;

  private final CANCoder m_turnEncoder;
  // Driving encoder uses the integrated FX encoder
  // e.g. testMotor.getSelectedSensorPosition();


  // PID controller for velocity. DO NOT SET kD.  It is redundant as setVoltage() already controls this
  private final PIDController m_drivePIDController =
      new PIDController(
          ModuleConstants.kPModuleDriveController,
          ModuleConstants.kIModuleDriveController, // 0
          ModuleConstants.kDModuleDriveController // 0
      );

  // ShuffleboardTab PIDtab = Shuffleboard.getTab("PID Tuning");


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
      DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);

  SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(
      DriveConstants.ksTurning, DriveConstants.kvTurning, DriveConstants.kaTurning);

  private boolean done = false;

  // shuffleboard stuff
  ShuffleboardLayout shuffleboardContainer;

  /**
   * Constructs a swerve module
   * @param driveMotorChannel ID of the drive motor
   * @param turningMotorChannel ID of the turn motor
   * @param turningEncoderChannel ID of the CANCoder
   * @param angleZero CANCoder offset
   * @param encoderReversed is the turn encoder reversed
   * @param driveReversed is the drive motor reversed
   * @param container shuffleboard container to print debug to
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double angleZero,
      boolean encoderReversed,
      boolean driveReversed,
      ShuffleboardLayout container
      ) {

    // Initialize the motors
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turningMotor = new WPI_TalonFX(turningMotorChannel);

    // For testing, can be removed later
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);

    // Handle whether motor should be reversed or not
    m_driveMotor.setInverted(driveReversed);
    m_turningMotor.setInverted(true);

    // Configure the encoders for both motors
    m_driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    m_turnEncoder = new CANCoder(turningEncoderChannel);
    m_turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_turnEncoder.configMagnetOffset(angleZero);
    m_turnEncoder.configSensorDirection(encoderReversed);
    m_turnEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);
    m_turnEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 250);

    m_turningMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    m_turningMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);

    m_driveMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    m_driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // Code for current limiting
    m_driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 65, 0.1));
    m_driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.1));
    m_turningMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 65, 0.1));
    m_turningMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.1));

    // Shuffleboard
    shuffleboardContainer = container;
  }


  /**
   * Gets the heading of the module
   * @return the absolute position of the CANCoder
   */
  public double getModuleHeading(){
    return this.m_turnEncoder.getAbsolutePosition() % 360;
  }

  /**
   * Returns the current state of the module.
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    double m_speedMetersPerSecond =
        ModuleConstants.kDrivetoMetersPerSecond * m_driveMotor.getSelectedSensorVelocity();

    double m_turningRadians =
        (Math.PI / 180) * m_turnEncoder.getAbsolutePosition();

    return new SwerveModuleState(m_speedMetersPerSecond, new Rotation2d(m_turningRadians));
  }


  /**
   * Sets the desired state for the module and sends calculated output from controller to the motor.
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    double m_speedMetersPerSecond =
        ModuleConstants.kDrivetoMetersPerSecond * m_driveMotor.getSelectedSensorVelocity();

    double m_turnRadians =
        ((2 * Math.PI)/360) * m_turnEncoder.getAbsolutePosition();

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turnRadians));

//    stop the code to stop it from moving if the speed is very, very small
    if (Math.abs(state.speedMetersPerSecond) <= 0.01){
      m_turningMotor.set(0);
      m_driveMotor.set(0);
      return;
    }

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_speedMetersPerSecond, state.speedMetersPerSecond)
             + driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turnPIDController.calculate(m_turnRadians, state.angle.getRadians())
            + turnFeedForward.calculate(m_turnPIDController.getSetpoint().velocity);
    m_driveMotor.set(driveOutput/12);
    m_turningMotor.set(turnOutput/12);
  }

  /**
   * Gets the current position of the CANCoder
   * @return cancoder position with magnet offset
   */
  public double getCANCoder(){
    return m_turnEncoder.getPosition();
  }

  /**
   * Gets the current position of the CANCoder in relation to the magnet
   * @return current CANCoder position
   */
  public double getCANCoderABS(){
    return m_turnEncoder.getAbsolutePosition();
  }

  @Deprecated
  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
      m_turnEncoder.setPosition(0);
      m_driveMotor.setSelectedSensorPosition(0);
  }

  public void periodic_func() {
//    SmartDashboard.putNumber(shuffleboardContainer.getTitle() + " ABS", m_turnEncoder.getAbsolutePosition());
//    SmartDashboard.putNumber(shuffleboardContainer.getTitle() + " pos", m_turnEncoder.getPosition());
  }


}
