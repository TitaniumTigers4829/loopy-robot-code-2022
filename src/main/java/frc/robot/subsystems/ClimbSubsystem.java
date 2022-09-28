package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElectronicsConstants;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// TODO: Test to find out what voltage is needed to climb slowly, and what voltage is needed to hold position at bottom.

public class ClimbSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_leftMotor;
  private final WPI_TalonFX m_rightMotor;

  private final CANCoder m_leftEncoder;
  private final CANCoder m_rightEncoder;

  private final DigitalInput m_leftLimitSwitch;
  private final DigitalInput m_rightLimitSwitch;

  private final DoubleSolenoid m_solenoid;

  private NetworkTableInstance tableInstance;
  private NetworkTable baseTable;
  private NetworkTableEntry leftClimbHookZeroEntry;
  private NetworkTableEntry leftClimbHookHeightEntry;
  private NetworkTableEntry rightClimbHookZeroEntry;
  private NetworkTableEntry rightClimbHookHeightEntry;
  private NetworkTableEntry leftZeroReceivedSuccessfully;
  private NetworkTableEntry rightZeroReceivedSuccessfully;

  private double leftClimbHookZero = 1;
  private double rightClimbHookZero = 1;

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

  private final SimpleMotorFeedforward m_leftFeedForward = new SimpleMotorFeedforward(
      0.3,
      8,
      0
  );
  private final SimpleMotorFeedforward m_rightFeedForward = new SimpleMotorFeedforward(
      0.3,
      8,
      0
  );

  /**
   * Creates the climb subsystem.
   */
  public ClimbSubsystem() {
    // Initialize Motors
    m_leftMotor = new WPI_TalonFX(ClimbConstants.kLeftClimbMotorPort);
    m_rightMotor = new WPI_TalonFX(ClimbConstants.kRightClimbMotorPort);

    // Initialize Encoders
    m_leftEncoder = new CANCoder(ClimbConstants.kLeftClimbEncoderPort);
    m_rightEncoder = new CANCoder(ClimbConstants.kRightClimbEncoderPort);

    m_leftEncoder.configSensorDirection(true);

    m_leftEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);
    m_rightEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);

    m_leftMotor.setInverted(true);
    m_rightMotor.setInverted(true);

    m_leftMotor.setNeutralMode(NeutralMode.Brake);
    m_rightMotor.setNeutralMode(NeutralMode.Brake);

    // Initialize Limit Switches
    m_leftLimitSwitch = new DigitalInput(ClimbConstants.kLeftClimbLimitSwitchPort);
    m_rightLimitSwitch = new DigitalInput(ClimbConstants.kRightClimbLimitSwitchPort);

    // Initialize Solenoid
    m_solenoid = new DoubleSolenoid(ElectronicsConstants.kPneumaticsModuleType,
        ClimbConstants.kClimbVerticalSolenoidPort, ClimbConstants.kClimbAngledSolenoidPort);

    // Network Table Setup
    this.tableInstance = NetworkTableInstance.getDefault();
    this.baseTable = this.tableInstance.getTable("climbZerosTable");
    this.leftClimbHookHeightEntry = this.baseTable.getEntry("leftClimbHookHeight");
    this.leftClimbHookZeroEntry = this.baseTable.getEntry("leftClimbHookZero");
    this.rightClimbHookHeightEntry = this.baseTable.getEntry("rightClimbHookHeight");
    this.rightClimbHookZeroEntry = this.baseTable.getEntry("rightClimbHookZero");
    this.leftZeroReceivedSuccessfully = this.baseTable.getEntry("leftZeroReceivedSuccessfully");
    this.rightZeroReceivedSuccessfully = this.baseTable.getEntry("rightZeroReceivedSuccessfully");

    this.leftClimbHookHeightEntry.setDouble(0.5);
    this.rightClimbHookHeightEntry.setDouble(0.5);
  }

  public double getLeftClimbHookZeroValue() {
    return this.leftClimbHookZero;
  }

  public void setLeftClimbHookZeroValue(double value) {
    this.leftClimbHookZero = value;
  }

  public double getRightClimbHookZeroValue() {
    return this.rightClimbHookZero;
  }

  public void setRightClimbHookZeroValue(double value) {
    this.rightClimbHookZero = value;
  }

  /**
   * Returns encoder values, but zeroing has already been taken care of.
   *
   * @return encoder value
   */
  public double getLeftEncoderValue() {
    return m_leftEncoder.getPosition();
  }

  /**
   * Gets whether the left limit switch is triggered. When triggered, it indicates that the climb is
   * at its lowest possible position.
   *
   * @return true: climb is at bottom pos, false: climb is not at bottom pos
   */
  public boolean getIsLeftLimitSwitchPressed() {
    return m_leftLimitSwitch.get();
  }

  /**
   * Returns left hook height (defined as the uppermost part of the inner edge of the hook) in
   * meters. Uses encoder values to calculate the height of the hooks. If limit switch is triggered
   * then we know the hooks are at the bottom position. Because the desired state of the PID loop
   * should be the min height, this should automatically stop the motor.
   *
   * @return height of left hook (in meters)
   */
  public double getLeftHookHeight() {
    double multiplier = (ClimbConstants.kClimbMaxHeight - ClimbConstants.kClimbMinHeight) / (0
        - ClimbConstants.kClimbLeftMinHeightEncoderEstimate);
    if (!getIsLeftLimitSwitchPressed()) {
      return multiplier * getLeftEncoderValue()
          + ClimbConstants.kClimbMaxHeight;
    } else {
      return ClimbConstants.kClimbMinHeight;
    }
  }

  public double getLeftHookHeightNoLimits() {
    double multiplier = (ClimbConstants.kClimbMaxHeight - ClimbConstants.kClimbMinHeight)
        / (0 - ClimbConstants.kClimbLeftMinHeightEncoderEstimate);
    return multiplier * getLeftEncoderValue()
        + ClimbConstants.kClimbMaxHeight;
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
   * Gets whether the right limit switch is triggered. When triggered, it indicates that the climb
   * is at its lowest possible position.
   *
   * @return true: climb is at bottom pos, false: climb is not at bottom pos
   */
  public boolean getIsRightLimitSwitchPressed() {
    return !m_rightLimitSwitch.get();
  }

  /**
   * Returns right hook height (defined as the uppermost part of the inner edge of the hook) in
   * meters. Uses encoder values to calculate the height of the hooks. If limit switch is triggered
   * then we know the hooks are at the bottom position. Because the desired state of the PID loop
   * should be the min height, this should automatically stop the motor.
   *
   * @return height of right hook (in meters)
   */
  public double getRightHookHeight() {
    double multiplier = (ClimbConstants.kClimbMaxHeight - ClimbConstants.kClimbMinHeight)
        / (0 - ClimbConstants.kClimbRightMinHeightEncoderEstimate);
    if (!getIsRightLimitSwitchPressed()) {
      return multiplier * getRightEncoderValue()
          + ClimbConstants.kClimbMaxHeight;
    } else {
      return ClimbConstants.kClimbMinHeight;
    }
  }
  public double getRightHookHeightNoLimits() {
    double multiplier = (ClimbConstants.kClimbMaxHeight - ClimbConstants.kClimbMinHeight)
        / (0 - ClimbConstants.kClimbRightMinHeightEncoderEstimate);
    return multiplier * getRightEncoderValue()
        + ClimbConstants.kClimbMaxHeight;
  }
  public double getLeftPIDError() {
    return m_climbLeftProfiledPIDController.getPositionError();
  }

  public double getRightPIDError() {
    return m_climbRightProfiledPIDController.getPositionError();
  } 

  /**
   * Gets whether the climb is vertical or not. Also handles unexpected states and throws exceptions
   * accordingly.
   *
   * @return isVertical (boolean)
   */
  public boolean getIsClimbVertical() {
    if (m_solenoid.get() == Value.kOff) {
      DriverStation.reportWarning("The climb solenoid should never be off. Something is wrong.",
          true);
    } else if (m_solenoid.get() == Value.kForward) {
      return true;
    } else if (m_solenoid.get() == Value.kReverse) {
      return false;
    } else {
      DriverStation.reportWarning(
          "For some reason the climb solenoid is in none of the 3 possible positions.", true);
    }
    return true;
  }

  /**
   * Sets left motor output for manual control and testing. Positive values extend arm, negative
   * values retract arm.
   *
   * @param output raw value (-1.0  to 1)
   */
  public void setLeftMotorOutputManual(double output) {
    m_leftMotor.set(output);
    this.leftClimbHookHeightEntry.setDouble(getLeftHookHeight());
  }

  /**
   * Sets right motor output for manual control and testing. Positive values extend arm, negative
   * values retract arm.
   *
   * @param output raw value (-1.0  to 1)
   */
  public void setRightMotorOutputManual(double output) {
    m_rightMotor.set(output);
    this.rightClimbHookHeightEntry.setDouble(getRightHookHeight());
  }


// haha jank code
  // private double getMotorOutput(double desiredHeight, double currentHeight, ProfiledPIDController controller, SimpleMotorFeedforward ff) {
    // double errorOut = (desiredHeight - currentHeight) * controller.getP();
    // controller.calculate(currentHeight, desiredHeight); // all this does is set setpoint
    // return -(errorOut + ff.calculate(controller.getSetpoint().velocity));
  // }
  private double getMotorOutput(ProfiledPIDController controller, SimpleMotorFeedforward ff, double desired, double current) {
    controller.calculate(current, desired);
    return ((desired - current) * controller.getP()) + ff.calculate(controller.getGoal().velocity);
  }

  /**
   * Sets the desired height for the left hook and sends calculated output from the PID controller
   * to the motor.
   *
   * @param height desired hook height (meters)
   */
  public void setDesiredLeftHookHeight(double height) {
    // m_climbLeftProfiledPIDController.calculate(getLeftHookHeight(), height);
    double leftOutput = getMotorOutput(m_climbLeftProfiledPIDController, m_leftFeedForward, height, getLeftHookHeight());
    m_leftMotor.set(ControlMode.PercentOutput, leftOutput);
    // SmartDashboard.putNumber("Error", height - getLeftHookHeight());
    // SmartDashboard.putNumber("Out", leftOutput);
  }

  /**
   * Sets the desired height for the right hook and sends calculated output from the PID controller
   * to the motor.
   *
   * @param height desired hook height (meters)
   */
  public void setDesiredRightHookHeight(double height) {
    // m_climbRightProfiledPIDController.calculate(getRightHookHeight(), height);
    double rightOutput = getMotorOutput(m_climbRightProfiledPIDController, m_rightFeedForward, height, getRightHookHeight());
    m_rightMotor.set(ControlMode.PercentOutput, rightOutput);
    // SmartDashboard.putNumber("Error_R", height - getRightHookHeight());
    // SmartDashboard.putNumber("Out_R", rightOutput);
  }

  public void setPos(double height) {
    setDesiredLeftHookHeight(height);
    setDesiredRightHookHeight(height);
  }

  /**
   * Sets left hook to bottom position
   */
  public void setLeftHookToBottomPos() {
    final double leftOutput = m_climbLeftProfiledPIDController.calculate(getLeftHookHeight(),
        ClimbConstants.kClimbMinHeight);
    setHookToBottomPos(leftOutput, m_climbLeftProfiledPIDController.getPositionError(),
        m_leftMotor);
  }

  /**
   * Sets right hook to bottom position
   */
  public void setRightHookToBottomPos() {
    final double rightOutput = m_climbRightProfiledPIDController.calculate(getRightHookHeight(),
        ClimbConstants.kClimbMinHeight);
    setHookToBottomPos(rightOutput, m_climbRightProfiledPIDController.getPositionError(),
        m_rightMotor);
//    m_rightMotor.set(rightOutput);
  }

  /**
   * Handles how we move a hook to the bottom position while climbing. Makes use of PID and limit
   * switches. Uses limit switches because when the limit switches get hit, the error reported by
   * the PID loop will be zero. This is because getLeftHookHeight returns
   * ClimbConstants.kClimbMinHeight when the limit switch is hit.
   *
   * @param outputFromPID What does the PID loop suggest we do?
   * @param errorFromPID  What is the error according to the PID controller?
   * @param motor         Which motor do we need to control?
   */
  private void setHookToBottomPos(double outputFromPID,
      double errorFromPID, WPI_TalonFX motor) {
    // FIXME: .getPositionError might need to be negated
    if (errorFromPID <= ClimbConstants.kClimbMinPosPIDErrorThreshold
        // Is error small, i.e. are we close?
        && errorFromPID != 0) { // But not fully there yet?
      motor.set(ClimbConstants.kClimbVoltageToApplyAfterPID / 12);
    } else if (errorFromPID
        > ClimbConstants.kClimbMinPosPIDErrorThreshold) { // We must not be close
      motor.set(outputFromPID); // Let the PID loop do the work
    } else { // we must be touching the limit switch
      motor.set(ClimbConstants.kClimbVoltageToHoldBottomPosition / 12); // Hold position
    }
  }

  /**
   * Sets left hook to full extension. Makes setpoint a little higher than it needs to be to ensure
   * that arms are all the way extended. All this does is loosen the rope on the spool a tiny bit,
   * but provides nice to have peace of mind.
   * TODO: confirm that this is within frame perimeter when angled.
   */
  public void setLeftHookToFullExtension() {
    setDesiredLeftHookHeight(
        ClimbConstants.kClimbMaxHeight + ClimbConstants.kClimbMaxPosConfirmationExtraHeight);
  }

  /**
   * Sets right hook to full extension. Makes setpoint a little higher than it needs to be to ensure
   * * that arms are all the way extended. All this does is loosen the rope on the spool a tiny bit,
   * * but provides nice to have peace of mind.
   * TODO: confirm that this is within frame perimeter when angled.
   */
  public void setRightHookToFullExtension() {
    setDesiredRightHookHeight(
        ClimbConstants.kClimbMaxHeight + ClimbConstants.kClimbMaxPosConfirmationExtraHeight);
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

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    double leftValue = this.leftClimbHookZeroEntry.getDouble(-1);
    double rightValue = this.rightClimbHookZeroEntry.getDouble(-1);

    if (leftValue != -1) {
      setLeftClimbHookZeroValue(leftValue);
    }
    if (rightValue != -1) {
      setRightClimbHookZeroValue(leftValue);
    }

    // SmartDashboard.putNumber("Left climb zero", getLeftClimbHookZeroValue());
    // SmartDashboard.putNumber("Right climb zero", getRightClimbHookZeroValue());
    // Warn Drivers if the hooks are not near each other for some reason.
//    if (Math.abs(getLeftHookHeight() - getRightHookHeight()) > 0.25) {
//      DriverStation.reportWarning(
//          "The climb hooks are not in sync. Manual control might be needed.", false);
//    }
    // Smart Dashboard Debugging
    // SmartDashboard.putBoolean("Left Climb Limit Switch: ", getIsLeftLimitSwitchPressed());
    // SmartDashboard.putBoolean("Right Climb Limit Switch: ", getIsRightLimitSwitchPressed());
//    SmartDashboard.putBoolean("Right Climb Limit Switch: ", getIsRightLimitSwitchPressed());
  //  SmartDashboard.putNumber("Left Climb Encoder: ", getLeftEncoderValue());
//    SmartDashboard.putNumber("Right Climb Encoder: ", getRightEncoderValue());
    // SmartDashboard.putNumber("Left Climb PID Error: ", getLeftPIDError());
//    SmartDashboard.putNumber("Right Climb PID Error: ", getRightPIDError());
//    SmartDashboard.putNumber("Left Climb PID setpoint: ", m_climbLeftProfiledPIDController.getGoal().position);
//    SmartDashboard.putNumber("Right Climb PID setpoint: ", m_climbRightProfiledPIDController.getGoal().position);

//    SmartDashboard.putNumber("Left Climb PID velocity: ", m_climbLeftProfiledPIDController.getGoal().velocity);
//    SmartDashboard.putNumber("Right Climb PID velocity: ", m_climbRightProfiledPIDController.getGoal().velocity);
  
//   SmartDashboard.putNumber("Left Hook Height: ", getLeftHookHeight());
  //  SmartDashboard.putNumber("Right Hook Height:", getRightHookHeight());
  
//    SmartDashboard.putBoolean("Is Climb Vertical?: ", getIsClimbVertical());
  }
}
