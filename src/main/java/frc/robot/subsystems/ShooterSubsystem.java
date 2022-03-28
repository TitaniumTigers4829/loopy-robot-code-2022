package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_leftMotor;
  private final WPI_TalonFX m_rightMotor;

  private double speed = 0;

  // TODO: IDEA: turn off the compressor while trying to shoot to help the limelight see better
  /** Creates the Shooter subsystem. */
  public ShooterSubsystem() {
    // Initialize Motors
    m_leftMotor = new WPI_TalonFX(ShooterConstants.kLeftShooterMotorPort);
    m_rightMotor = new WPI_TalonFX(ShooterConstants.kRightShooterMotorPort);

    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(true);

    m_leftMotor.setNeutralMode(NeutralMode.Coast);
    m_rightMotor.setNeutralMode(NeutralMode.Coast);
    m_leftMotor.enableVoltageCompensation(true);
    m_rightMotor.enableVoltageCompensation(true);
    m_leftMotor.configVoltageCompSaturation(12);
    m_rightMotor.configVoltageCompSaturation(12);

    // configure built-in encoder (only need to use the right one).
    m_rightMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
  }

  /**
   * Sets speed of fly wheels
   * @param speed has to be in between -1 and 1
   */
  public void setSpeed(double speed) {
    this.speed = speed;
    m_leftMotor.set(speed);
    m_rightMotor.set(speed);
  }

  public void increaseSpeed() {
    speed += .05;
    speed = (speed >= 1 ? 1 : speed);
    m_rightMotor.set(speed);
    m_leftMotor.set(speed);
  }

  public void decreaseSpeed() {
    speed -= .05;
    speed = (speed <= -1 ? -1 : speed);
    m_rightMotor.set(speed);
    m_leftMotor.set(speed);
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

