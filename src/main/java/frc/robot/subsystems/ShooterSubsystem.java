package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_frontMotor;
  private final WPI_TalonFX m_backMotor;

  private double speed = 0;

  // TODO: IDEA: turn off the compressor while trying to shoot to help the limelight see better
  /** Creates the Shooter subsystem. */
  public ShooterSubsystem() {
    // Initialize Motors
    m_frontMotor = new WPI_TalonFX(ShooterConstants.kBackShooterMotorPort);
    m_backMotor = new WPI_TalonFX(ShooterConstants.kFrontShooterMotorPort);

    m_frontMotor.setInverted(true);
    m_backMotor.setInverted(true);

    m_frontMotor.setNeutralMode(NeutralMode.Coast);
    m_backMotor.setNeutralMode(NeutralMode.Coast);
    m_frontMotor.enableVoltageCompensation(true);
    m_backMotor.enableVoltageCompensation(true);
    m_frontMotor.configVoltageCompSaturation(12);
    m_backMotor.configVoltageCompSaturation(12);

    // configure built-in encoder (only need to use the right one).
    m_backMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
  }

  /**
   * Sets speed of fly wheels
   * @param speed has to be in between -1 and 1
   */
  public void setSpeed(double speed) {
    this.speed = speed;
    m_frontMotor.set(speed);
    m_backMotor.set(speed);
  }

  public void increaseSpeed() {
    speed += .05;
    speed = (speed >= 1 ? 1 : speed);
    m_backMotor.set(speed);
    m_frontMotor.set((speed * 1.2 >= 1 ? 1 : speed * 1.2));
    SmartDashboard.putNumber("Speed", speed);
  }

  public void decreaseSpeed() {
    speed -= .05;
    speed = (speed <= -1 ? -1 : speed);
    m_backMotor.set(speed);
    m_frontMotor.set((speed * 1.2 <= -1 ? -1 : speed * 1.2));
    SmartDashboard.putNumber("Speed", speed);
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

