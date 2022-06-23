// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TowerConstants;


public class TowerSubsystem extends SubsystemBase {
  private final WPI_TalonSRX m_topTowerMotor;
  private final WPI_TalonSRX m_bottomTowerMotor;

  private final DigitalInput bottomTowerSensor;
  private final DigitalInput topTowerSensor;

  public TowerSubsystem() {
    m_topTowerMotor = new WPI_TalonSRX(TowerConstants.topTowerFeedMotorPort);
    m_bottomTowerMotor = new WPI_TalonSRX(TowerConstants.bottomTowerFeedMotorPort);

    bottomTowerSensor = new DigitalInput(TowerConstants.bottomTowerSensorPort);
    topTowerSensor = new DigitalInput(TowerConstants.topTowerSensorPort);

    m_topTowerMotor.configFactoryDefault();
    m_bottomTowerMotor.configFactoryDefault();

    m_bottomTowerMotor.setNeutralMode(NeutralMode.Brake);
    m_topTowerMotor.setNeutralMode(NeutralMode.Brake);

    m_bottomTowerMotor.setInverted(true);
    m_topTowerMotor.setInverted(true);

    m_bottomTowerMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    m_bottomTowerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);

    m_topTowerMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    m_topTowerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);
  }

  public void setTowerMotorsSpeed(double speed) {
    m_topTowerMotor.set(speed);
    m_bottomTowerMotor.set(speed);
  }

  public void setTopMotorOutputManual(double output) {
    m_topTowerMotor.set(output);
  }

  public void setBottomMotorOutputManual(double output) {
    m_bottomTowerMotor.set(output);
  }

  public void setTowerFullPower() {
    m_topTowerMotor.set(1.0);
    m_bottomTowerMotor.set(1.0);
  }

  public void setTowerThirdPower() {
    m_topTowerMotor.set(0.334);
    m_bottomTowerMotor.set(0.34);
  }

  public void setTowerOff() {
    m_topTowerMotor.set(0);
    m_bottomTowerMotor.set(0);
  }

  public boolean getIsBallInBottom(){
//    return false;
    return !bottomTowerSensor.get();
  }

  public boolean getIsBallInTop(){
//    return false;
    return !topTowerSensor.get();
  }

  @Override
  public void periodic() {
//    SmartDashboard.putBoolean("Bottom Tower Sensor", getIsBallInBottom());
//    SmartDashboard.putBoolean("Top Tower Sensor", getIsBallInTop());
  }
}
