// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class NewShooterSubsystem extends SubsystemBase {

  private final WPI_TalonFX topMotor = new WPI_TalonFX(ShooterConstants.kTopShooterMotorPort);
  private final WPI_TalonFX bottomMotor = new WPI_TalonFX(ShooterConstants.kBottomShooterMotorPort);

  private final SimpleMotorFeedforward topMotorFeedforward = 
    new SimpleMotorFeedforward(ShooterConstants.topkS, ShooterConstants.topkV);
  private final SimpleMotorFeedforward bottomMotorFeedforward = 
    new SimpleMotorFeedforward(ShooterConstants.bottomkS, ShooterConstants.bottomkV);

  public NewShooterSubsystem() {
    // Motor Setup
    topMotor.configFactoryDefault();
    bottomMotor.configFactoryDefault();

    topMotor.setInverted(true);
    bottomMotor.setInverted(true);

    topMotor.setNeutralMode(NeutralMode.Coast);
    bottomMotor.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
