// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TowerConstants;

/*
1 button - spin intake, bottom tower, upper tower
when ball detected by photoelectric sensor, stop top tower
*/

public class TowerSubsystem extends SubsystemBase {
  /** Creates a new Turret. */

  private final WPI_TalonSRX topTowerMotor;
  private final WPI_TalonSRX bottomTowerMotor;
//    private final WPI_TalonFX turretMotor;

  private final DigitalInput bottomTowerSensor;
  private final DigitalInput topTowerSensor;

//    private final LinearInterpolator angleInterpolator;

  public TowerSubsystem() {
    topTowerMotor = new WPI_TalonSRX(TowerConstants.topTowerFeedMotorPort);
    bottomTowerMotor = new WPI_TalonSRX(TowerConstants.bottomTowerFeedMotorPort);
//        turretMotor = new WPI_TalonFX(MotorConstants.shootMotorID);

//        angleInterpolator = new LinearInterpolator(TurretConstants.angleTable);

    bottomTowerSensor = new DigitalInput(TowerConstants.bottomTowerFeedMotorPort);
    topTowerSensor = new DigitalInput(TowerConstants.topTowerFeedMotorPort);

    topTowerMotor.configFactoryDefault();
    bottomTowerMotor.configFactoryDefault();

    bottomTowerMotor.setNeutralMode(NeutralMode.Brake);
    topTowerMotor.setNeutralMode(NeutralMode.Brake);

    bottomTowerMotor.setInverted(true);
    topTowerMotor.setInverted(true);

//        actuator1 = new Servo(MotorConstants.actuator1Port);
//        actuator2 = new Servo(MotorConstants.actuator2Port);
  }

  public boolean getSensor(){
//        return sensor.get();
    return false;
  }

  public void setTurretAngle(double angle){
//        angle = angleInterpolator.getInterpolatedValue(angle);
//        actuator1.set(angle);
//        actuator2.set(angle);
  }

  public void setTurretRatio(double ratioPos){
//        actuator1.set(ratioPos);
//        actuator2.set(ratioPos);
  }
  /** Set turret pos in % of extension
   * @param extension percent of extension
   */
  public void setTurretExtension(double extension){
//        actuator1.set(extension / 100);
//        actuator2.set(extension / 100);
  }

//  public void spinUntilBall(){
//    boolean isBallIn = false;
//    while (!isBallIn){
//      bottomTowerMotor.set(ControlMode.PercentOutput, 0.2);
//      topTowerMotor.set(ControlMode.PercentOutput, 0.2);
////            isBallIn = sensor.get();
//    }
//    topTowerMotor.set(ControlMode.PercentOutput, 0);
//    bottomTowerMotor.set(ControlMode.PercentOutput, 0.5);
//  }

  public void stop(){
//        intakeMotor.set(ControlMode.PercentOutput, 0.0);
    topTowerMotor.set(ControlMode.PercentOutput, 0.0);
    bottomTowerMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void pointBlank(){

  }

//    public void shoot(){
//        turretMotor.set(ControlMode.PercentOutput, 0.5);
//    }
//
//    public void stopShooting(){
//        turretMotor.set(ControlMode.PercentOutput, 0.0);
//    }

  public void testTower(double speed){
    if (Math.abs(speed) <= 0.05){
      bottomTowerMotor.set(0);
      topTowerMotor.set(0);
      return;
    }
    bottomTowerMotor.set(speed);
    topTowerMotor.set(speed);
    SmartDashboard.putNumber("SPEED", speed);
  }

  public void stopTesting(){
    bottomTowerMotor.set(0);
    topTowerMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Bottom Tower Sensor", bottomTowerSensor.get());
    SmartDashboard.putBoolean("Top Tower Sensor", topTowerSensor.get());
  }
}
