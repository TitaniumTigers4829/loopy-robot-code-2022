package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_topMotor;
  private final WPI_TalonFX m_bottomMotor;

  private double speed = 0;

  // TODO: IDEA: turn off the compressor while trying to shoot to help the limelight see better
  /** Creates the Shooter subsystem. */
  public ShooterSubsystem() {
    // Initialize Motors
    m_topMotor = new WPI_TalonFX(ShooterConstants.kBottomShooterPort);
    m_bottomMotor = new WPI_TalonFX(ShooterConstants.kTopShooterPort);

    m_topMotor.setInverted(true);
    m_bottomMotor.setInverted(true);

    m_topMotor.setNeutralMode(NeutralMode.Coast);
    m_bottomMotor.setNeutralMode(NeutralMode.Coast);
    m_topMotor.configVoltageCompSaturation(12);
    m_bottomMotor.configVoltageCompSaturation(12);
    m_topMotor.enableVoltageCompensation(true);
    m_bottomMotor.enableVoltageCompensation(true);

    // configure built-in encoder (only need to use the right one).
    m_bottomMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
  }

  /**
   * Sets speed of fly wheels
   * @param speed has to be in between -1 and 1
   */
  public void setSpeed(double speed) {
    this.speed = speed;
    m_topMotor.set(speed / 1.2);
    m_bottomMotor.set(speed);
  }

//  public void increaseBackSpeed() {
//    speed += .05;
//    speed = (speed >= 1 ? 1 : speed);
//    m_bottomMotor.set(speed);
//    SmartDashboard.putNumber("back speed", speed);
//  }
//
//  public void decreaseBackSpeed() {
//    speed -= .05;
//    speed = (speed <= -1 ? -1 : speed);
//    m_bottomMotor.set(speed);
//    SmartDashboard.putNumber("back speed", speed);
//  }
//
//  public void increaseFrontSpeed() {
//    speed += .05;
//    speed = (speed >= 1 ? 1 : speed);
//    m_topMotor.set(speed);
//    SmartDashboard.putNumber("front speed", speed);
//  }
//
//  public void decreaseFrontSpeed() {
//    speed -= .05;
//    speed = (speed <= -1 ? -1 : speed);
//    m_topMotor.set(speed);
//    SmartDashboard.putNumber("front speed", speed);
//  }

  public void setShooterRPM(double topSpeed, double bottomSpeed) {
//    targetRPM = speedMain;
//    // 2048 ticks per revolution, ticks per .10 second, 1 / 2048 * 60
//    double speed_FalconUnits1 = speedMain / (600.0) * 2048.0;
//    double speed_FalconUnits2 = speedTop / (600.0) * 2048.0;
//
//    if (Math.abs(getMainRPM()) < Math.abs(speedMain) * 1.1) {
//      FlywheelMain.set(TalonFXControlMode.Velocity, speed_FalconUnits1);
//    } else {
//      FlywheelMain.set(ControlMode.PercentOutput, 0);
//    }
//
//    if (Math.abs(getTopRPM()) < Math.abs(speedTop) * 1.1) {
//      FlywheelTop.set(TalonFXControlMode.Velocity, speed_FalconUnits2);
//    } else {
//      FlywheelTop.set(ControlMode.PercentOutput, 0);
//    }
  }

  public double getTopRPM(){
    return (m_topMotor.getSelectedSensorVelocity()) / 2048 * 600;
  }

  public double getBottomRPM(){
    return (m_bottomMotor.getSelectedSensorVelocity()) / 2048 * 600;
  }

  @Override
  public void periodic() {
    // Smart Dashboard Debugging
//    SmartDashboard.putNumber("Shooter Encoder Value:", m_rightMotor.getSelectedSensorPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

