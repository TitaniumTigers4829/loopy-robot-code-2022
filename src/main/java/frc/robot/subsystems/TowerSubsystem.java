// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TowerConstants;


public class TowerSubsystem extends SubsystemBase {
  private final WPI_TalonSRX topTowerMotor;
  private final WPI_TalonSRX bottomTowerMotor;

//  private final DigitalInput bottomTowerSensor;
//  private final DigitalInput topTowerSensor;

  public TowerSubsystem() {
    topTowerMotor = new WPI_TalonSRX(TowerConstants.topTowerFeedMotorPort);
    bottomTowerMotor = new WPI_TalonSRX(TowerConstants.bottomTowerFeedMotorPort);

//    bottomTowerSensor = new DigitalInput(TowerConstants.bottomTowerFeedMotorPort);
//    topTowerSensor = new DigitalInput(TowerConstants.topTowerFeedMotorPort);

    topTowerMotor.configFactoryDefault();
    bottomTowerMotor.configFactoryDefault();

    bottomTowerMotor.setNeutralMode(NeutralMode.Brake);
    topTowerMotor.setNeutralMode(NeutralMode.Brake);

    bottomTowerMotor.setInverted(true);
    topTowerMotor.setInverted(true);
  }

  public void setTopMotorOutputManual(double output) {
    topTowerMotor.set(output);
  }

  public void setBottomMotorOutputManual(double output) {
    bottomTowerMotor.set(output);
  }

  public void setTowerFullPower() {
    topTowerMotor.set(1.0);
    bottomTowerMotor.set(1.0);
  }

  public void setTowerThirdPower() {
    topTowerMotor.set(0.334);
    bottomTowerMotor.set(0.34);
  }

  public void setTowerOff() {
    topTowerMotor.set(0);
    bottomTowerMotor.set(0);
  }

  @Override
  public void periodic() {
//    SmartDashboard.putBoolean("Bottom Tower Sensor", bottomTowerSensor.get());
//    SmartDashboard.putBoolean("Top Tower Sensor", topTowerSensor.get());
  }
}
