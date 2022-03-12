package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElectronicsConstants;

// TODO: When make auto climb command, make sure to implement checking that the two arms are at least at similar heights, else throw an error.

public class ClimbSubsystem extends SubsystemBase{
  private final WPI_TalonFX m_leftMotor;
  private final WPI_TalonFX m_rightMotor;

  private final CANCoder m_leftEncoder;
  private final CANCoder m_rightEncoder;

  private final DoubleSolenoid m_solenoid;

  /** NOTE:
   * According to the documentation, it is possible to use a single controller asynchronously,
   * but that it is not have any built in thread safety, and should only be done by advanced teams.
   * Thus, using two separate controllers is much easier.
   */

  private final ProfiledPIDController m_climbLeftProfiledPIDController =
      new ProfiledPIDController(
          ClimbConstants.kPClimbController,
          ClimbConstants.kIClimbController, // Always 0, do not change
          ClimbConstants.kPClimbController,
          new TrapezoidProfile.Constraints(
              ClimbConstants.kMaxClimbSpeedMetersPerSecond,
              ClimbConstants.kMaxClimbAccelerationMetersPerSecondSquared));

  private final ProfiledPIDController m_climbRightProfiledPIDController =
      new ProfiledPIDController(
          ClimbConstants.kPClimbController,
          ClimbConstants.kIClimbController, // Always 0, do not change
          ClimbConstants.kPClimbController,
          new TrapezoidProfile.Constraints(
              ClimbConstants.kMaxClimbSpeedMetersPerSecond,
              ClimbConstants.kMaxClimbAccelerationMetersPerSecondSquared));

  /** Creates the climb subsystem. */
  public ClimbSubsystem() {
    // Initialize Motors
    m_leftMotor = new WPI_TalonFX(ClimbConstants.kLeftClimbMotorPort);
    m_rightMotor = new WPI_TalonFX(ClimbConstants.kRightClimbMotorPort);

    m_leftMotor.setInverted(true);
    m_rightMotor.setInverted(false);

    m_leftMotor.setNeutralMode(NeutralMode.Brake);
    m_rightMotor.setNeutralMode(NeutralMode.Brake);

    // Initialize Encoders
    m_leftEncoder = new CANCoder(ClimbConstants.kLeftClimbEncoderPort);
    m_rightEncoder = new CANCoder(ClimbConstants.kRightClimbEncoderPort);

    // TODO: implement config stuff

    // Initialize Solenoid
    m_solenoid = new DoubleSolenoid(ElectronicsConstants.kPneumaticsModuleType, ClimbConstants.kClimbVerticalSolenoidPort, ClimbConstants.kClimbAngledSolenoidPort);
  }

  /** Returns encoder values, but zeroing has already been taken care of.
   * @return encoder value
   */
  private double getLeftEncoderValue() {
    return m_leftEncoder.getPosition();
  }

  /** Uses encoder values to calculate the height of the hooks.
   * @return height of left hook (meters)
   */
  public double getLeftHookHeight() {
    return 0;
  }

  /** Returns encoder values, but zeroing has already been taken care of.
   * @return encoder value
   */
  private double getRightEncoderValue() {
    return m_rightEncoder.getPosition();
  }

  /** Uses encoder values to calculate the height of the hooks.
   * @return height of right hook (meters)
   */
  public double getRightHookHeight() {
    return 0;
  }

  /** Uses hook height of each side to calculate the average hook height between the arms.
   * @return average height of the hooks (meters)
   */
  public double getAverageHookHeight() {
    return (getLeftHookHeight() + getRightHookHeight()) / 2;
  }

  /** Gets whether the climb is vertical or not.
   *  Also handles unexpected states and throws exceptions accordingly.
   *  @return isVertical (boolean)
   */
  public boolean getIsClimbVertical() {
    if (m_solenoid.get() == Value.kOff) {
      throw new RuntimeException("The climb solenoid should never be off. Something is wrong.");
    } else if (m_solenoid.get() == Value.kForward) {
      return true;
    } else if (m_solenoid.get() == Value.kReverse) {
      return false;
    } else {
      throw new RuntimeException("For some reason the climb solenoid is in none of the 3 possible positions.");
    }
  }


  /** Sets the desired height for the left hook and sends calculated output from controller to the motor.
   * @param height desired hook height (meters)
   */
  private void setDesiredLeftHookHeight(double height) {
    final double leftOutput =
  }

  /** Sets the desired height for the right hook and sends calculated output from controller to the motor.
   * @param height desired hook height (meters)
   */
  private void setDesiredRightHookHeight(double height) {

  }

  /** Sets the desired height for the hooks.
   * @param height desired hook height (meters)
   */
  public void setDesiredHookHeight(double height) {
    setDesiredLeftHookHeight(height);
    setDesiredRightHookHeight(height);
  }

  /** Instructs the solenoid to make climb vertical. */
  public void setClimbVertical() {
    m_solenoid.set(Value.kForward);
  }

  /** Instructs the solenoid to make climb angled. */
  public void setClimbAngled() {
    m_solenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // Smart Dashboard Debugging
    SmartDashboard.putNumber("Left Climb Encoder : ", getLeftEncoderValue());
    SmartDashboard.putNumber("Right Climb Encoder : ", getRightEncoderValue());

    SmartDashboard.putNumber("Right Hook Height : ", getLeftHookHeight());
    SmartDashboard.putNumber("Left Hook Height: ", getRightHookHeight());

    SmartDashboard.putBoolean("Is Climb Vertical?: ", getIsClimbVertical());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
