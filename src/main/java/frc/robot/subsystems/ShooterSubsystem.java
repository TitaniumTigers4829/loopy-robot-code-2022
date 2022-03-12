package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonFX m_leftMotor;
  private final WPI_TalonFX m_rightMotor;


  // TODO: IDEA: turn off teh compressor while trying to shoot to help the limelight see better
  /** Creates the Shooter subsystem. */
  public ShooterSubsystem() {

    // Initialize Motors
    m_leftMotor = new WPI_TalonFX(ShooterConstants.kLeftShooterMotorPort);
    m_rightMotor = new WPI_TalonFX(ShooterConstants.kRightShooterMotorPort);

    m_leftMotor.setInverted(true);
    m_rightMotor.setInverted(false);

    m_leftMotor.setNeutralMode(NeutralMode.Coast);
    m_rightMotor.setNeutralMode(NeutralMode.Coast);


    // configure built-in encoder (only need to use the right one).
    m_rightMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
  }

  public double getEncoderValue() {
    return m_rightMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // Smart Dashboard Debugging
    SmartDashboard.putNumber("Shooter Encoder Value:", m_rightMotor.getSelectedSensorPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

