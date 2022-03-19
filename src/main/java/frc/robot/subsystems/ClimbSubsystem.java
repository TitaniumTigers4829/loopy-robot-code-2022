package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElectronicsConstants;

// TODO: When make auto climb command, make sure to implement checking that the two arms are at least at similar heights, else throw an error.

public class ClimbSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_leftMotor;
  private final WPI_TalonFX m_rightMotor;

  private final CANCoder m_leftEncoder;
  private final CANCoder m_rightEncoder;

  private final DigitalInput m_leftLimitSwitch;
  private final DigitalInput m_rightLimitSwitch;

  private final DoubleSolenoid m_solenoid;

  /**
   * NOTE: According to the documentation, it is possible to use a single controller asynchronously,
   * but that it is not have any built in thread safety, and should only be done by advanced teams.
   * Thus, using two separate controllers is much easier. It will also help account for slight
   * mechanical differences between the two arms.
   */

  // Important: This PID Controller uses exclusively SI units.
  private final ProfiledPIDController m_climbLeftProfiledPIDController =
      new ProfiledPIDController(
          ClimbConstants.kPClimbController,
          ClimbConstants.kIClimbController,
          ClimbConstants.kDClimbController,
          new TrapezoidProfile.Constraints(
              ClimbConstants.kMaxClimbSpeedMetersPerSecond,
              ClimbConstants.kMaxClimbAccelerationMetersPerSecondSquared));

  // Important: This PID Controller uses exclusively SI units.
  private final ProfiledPIDController m_climbRightProfiledPIDController =
      new ProfiledPIDController(
          ClimbConstants.kPClimbController,
          ClimbConstants.kIClimbController,
          ClimbConstants.kDClimbController,
          new TrapezoidProfile.Constraints(
              ClimbConstants.kMaxClimbSpeedMetersPerSecond,
              ClimbConstants.kMaxClimbAccelerationMetersPerSecondSquared));

  /**
   * Creates the climb subsystem.
   */
  public ClimbSubsystem() {
    // Initialize Motors
    m_leftMotor = new WPI_TalonFX(ClimbConstants.kLeftClimbMotorPort);
    m_rightMotor = new WPI_TalonFX(ClimbConstants.kRightClimbMotorPort);

    m_leftMotor.configFactoryDefault();
    m_rightMotor.configFactoryDefault();

    m_leftMotor.setInverted(true);
    m_rightMotor.setInverted(false);

    m_leftMotor.setNeutralMode(NeutralMode.Brake);
    m_rightMotor.setNeutralMode(NeutralMode.Brake);

    // Initialize Encoders
    m_leftEncoder = new CANCoder(ClimbConstants.kLeftClimbEncoderPort);
    m_rightEncoder = new CANCoder(ClimbConstants.kRightClimbEncoderPort);

    m_leftEncoder.configMagnetOffset(ClimbConstants.kLeftClimbEncoderOffsetForTopPos);
    m_rightEncoder.configMagnetOffset(ClimbConstants.kRightClimbEncoderOffsetForTopPos);

    // Initialize Limit Switches
    m_leftLimitSwitch = new DigitalInput(ClimbConstants.kLeftClimbLimitSwitchPort);
    m_rightLimitSwitch = new DigitalInput(ClimbConstants.kRightClimbLimitSwitchPort);

    // Initialize Solenoid
    m_solenoid = new DoubleSolenoid(ElectronicsConstants.kPneumaticsModuleType,
        ClimbConstants.kClimbVerticalSolenoidPort, ClimbConstants.kClimbAngledSolenoidPort);
  }

  /**
   * Returns encoder values, but zeroing has already been taken care of.
   *
   * @return encoder value
   */
  private double getLeftEncoderValue() {
    return m_leftEncoder.getPosition();
  }

  /**
   * Uses encoder values to calculate the height of the hooks.
   *
   * @return height of left hook (meters)
   */

  private boolean getIsLeftLimitSwitchPressed() {
    return m_leftLimitSwitch.get();
  }

  public double getLeftHookHeight() {
    return 0.00020887562 * getLeftEncoderValue() + 0.86995;
  }

  /**
   * Returns encoder values, but zeroing has already been taken care of.
   *
   * @return encoder value
   */
  private double getRightEncoderValue() {
    return m_rightEncoder.getPosition();
  }

  /**
   * Uses encoder values to calculate the height of the hooks.
   *
   * @return height of right hook (meters)
   */
  public double getRightHookHeight() {
    return 0;
  }

  /**
   * Gets whether the climb is vertical or not. Also handles unexpected states and throws exceptions
   * accordingly.
   *
   * @return isVertical (boolean)
   */
  public boolean getIsClimbVertical() {
    if (m_solenoid.get() == Value.kOff) {
      DriverStation.reportWarning("The climb solenoid should never be off. Something is wrong.", true);
    } else if (m_solenoid.get() == Value.kForward) {
      return true;
    } else if (m_solenoid.get() == Value.kReverse) {
      return false;
    } else {
      DriverStation.reportWarning("For some reason the climb solenoid is in none of the 3 possible positions.", true);
    }
    return true;
  }

  /**
   * Sets left motor output for manual control and testing. NOTE: ONLY USE FOR TESTING.
   *
   * @param output raw value (-1.0  to 1)
   */
  public void setLeftMotorOutputManual(double output) {
    m_leftMotor.set(output);
  }

  /**
   * Sets right motor output for manual control and testing. NOTE: ONLY USE FOR TESTING.
   *
   * @param output raw value (-1.0  to 1)
   */
  public void setRightMotorOutputManual(double output) {
    m_rightMotor.set(output);
  }

  /**
   * Sets the desired height for the left hook and sends calculated output from controller to the
   * motor.
   *
   * @param height desired hook height (meters)
   */
  public void setDesiredLeftHookHeight(double height) {
    final double leftOutput = m_climbLeftProfiledPIDController.calculate(getLeftHookHeight(),
        height);
    m_leftMotor.set(leftOutput);
  }

  /**
   * Sets the desired height for the right hook and sends calculated output from controller to the
   * motor.
   *
   * @param height desired hook height (meters)
   */
  public void setDesiredRightHookHeight(double height) {
    final double rightOutput = m_climbRightProfiledPIDController.calculate(getRightHookHeight(),
        height);
    m_rightMotor.set(rightOutput);
  }

  public void setLeftHookToBottomPos() {
    // TODO: implement once we have the limit switches
    // TODO: Also make this reset the encoder math to account for weird rope behavior.
  }

  public void setRightHookToBottomPos() {
    // TODO: implement once we have the limit switches
    // TODO: Also make this reset the encoder math to account for weird rope behavior.
  }



  @Deprecated
  /**
   * Sets the desired height for the hooks.
   *
   * @param height desired hook height (meters)
   */
  public void setDesiredHookHeight(double height) {
    setDesiredLeftHookHeight(height);

    // TODO: uncomment this after initial testing.
    // setDesiredRightHookHeight(height);
  }

  /**
   * Instructs the solenoid to make climb vertical.
   */
  public void setClimbVertical() {
    m_solenoid.set(Value.kForward);
  }

  /**
   * Instructs the solenoid to make climb angled.
   */
  public void setClimbAngled() {
    m_solenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // Smart Dashboard Debugging
    //SmartDashboard.putNumber("Left Climb Encoder : ", getLeftEncoderValue());
//    SmartDashboard.putNumber("Right Climb Encoder : ", getRightEncoderValue());

    //SmartDashboard.putNumber("Left Hook Height : ", getLeftHookHeight());
    //SmartDashboard.putNumber("Right Hook Height: ", getRightHookHeight());

//    SmartDashboard.putBoolean("Is Climb Vertical?: ", getIsClimbVertical());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
