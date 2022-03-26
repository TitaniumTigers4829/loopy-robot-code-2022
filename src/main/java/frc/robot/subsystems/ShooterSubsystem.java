package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonFX m_leftMotor;
  private final WPI_TalonFX m_rightMotor;
  private final Servo m_leftServo;
  private final Servo m_rightServo;
  private double pos = 0;

  // TODO: IDEA: turn off the compressor while trying to shoot to help the limelight see better
  /** Creates the Shooter subsystem. */
  public ShooterSubsystem() {
    // Initialize Motors
    m_leftMotor = new WPI_TalonFX(ShooterConstants.kLeftShooterMotorPort);
    m_rightMotor = new WPI_TalonFX(ShooterConstants.kRightShooterMotorPort);
    m_leftServo = new Servo(ShooterConstants.kLeftServoPort);
    m_rightServo = new Servo(ShooterConstants.kRightServoPort);

    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(true);

    m_leftMotor.setNeutralMode(NeutralMode.Coast);
    m_rightMotor.setNeutralMode(NeutralMode.Coast);


    // configure built-in encoder (only need to use the right one).
    m_rightMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
  }

  public double getEncoderValue() {
    return m_rightMotor.getSelectedSensorPosition();
  }

  /**
   * Sets speed of fly wheels
   * @param speed has to be in between -1 and 1
   */
  public void setSpeed(double speed) {
    // TODO: Using only P + feed forward is probably the best idea
    m_leftMotor.set(speed);
    m_rightMotor.set(speed);
  }

  public void setHalfSpeed() {
    m_leftMotor.set(0.5);
    m_rightMotor.set(0.5);
  }

  public void setShooterFullSpeed() {
    m_leftMotor.set(1.0);
    m_rightMotor.set(1.0);
  }
//  FIXME
  public void setFenderShotHeight(){
    setHeight(ShooterConstants.fenderShotHeight);
  }

  public void increasePos(){
    pos += 0.05;
    setHeight(pos);
    SmartDashboard.putNumber("Extension", pos);
  }

  public void decreasePos(){
    pos -= 0.05;
    setHeight(pos);
    SmartDashboard.putNumber("Extension", pos);
  }

  public void stopShooter() {
    m_leftMotor.set(0);
    m_rightMotor.set(0);
  }

  /**
   * Sets the height of the tower
   * @param height Value should be in between 0 and 1
   */
  public void setHeight(double height) {
    m_leftServo.set(height);
    m_rightServo.set(height);
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

  public void setSpeed() {
  }
}

