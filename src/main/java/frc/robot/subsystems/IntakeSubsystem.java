package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectronicsConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_motor;
  private final DoubleSolenoid m_solenoid;
//  private final ColorSensorV3 colorSensor;
  private final int m_stallCounter = 1;

  public IntakeSubsystem() {

    m_motor = new WPI_TalonFX(IntakeConstants.kIntakeMotorPort);
    m_motor.setInverted(false); // should make sure motor.set(1.0) means we are intaking.
    m_motor.setNeutralMode(NeutralMode.Coast);
    m_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    m_solenoid = new DoubleSolenoid(ElectronicsConstants.kPneumaticsModuleType,
        IntakeConstants.kIntakeDeployedSolenoidPort, IntakeConstants.kIntakeRetractedSolenoidPort);

    m_motor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    m_motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);

//    colorSensor = new ColorSensorV3(Port.kMXP);
  }

//  public boolean getIsIntakeDeployed() {
//    if (m_solenoid.get() == Value.kOff) {
//      DriverStation.reportWarning("The intake solenoid should never be off. Something is wrong.", true);
//    } else if (m_solenoid.get() == Value.kForward) {
//      return true;
//    } else if (m_solenoid.get() == Value.kReverse) {
//      return false;
//    } else {
//      DriverStation.reportWarning("For some reason the intake solenoid is in none of the 3 possible positions.", true);
//    }
//    return true;
//  }

  public void setMotorStopped() {
    m_motor.set(0);
  }

  public void setMotorFullPowerIn() {
    m_motor.set(1);
  }

  public void setMotorFullPowerOut() {
    m_motor.set(-1);
  }

//
  public void setMotorCustomPower(double customPower) {
    m_motor.set(customPower);
  }
//
//  public void setMotorCustomPowerOut(double customPower) {
//    m_motor.set(-customPower);
//  }

  public void setSolenoidRetracted() {
    m_solenoid.set(Value.kReverse);
  }

  public void setSolenoidDeployed() {
    m_solenoid.set(Value.kForward);
  }

  //
  @Override
  public void periodic() {
//    // Smart Dashboard Debugging
//
//    // I believe motor should be reporting a max velocity theoretically close to 21777.0666666
//    // SO, if we detect drastically lower than that, its probably stalling.
//    if (getIntakeVelocity() < 10000) {
//      m_stallCounter++;
//    } else if ((getIntakeVelocity() >= 10000) && (m_stallCounter > 0)) {
//      m_stallCounter --;
//    }
//    // If the motor stall for longer than half a second, display warnings to the driver station.
//    if (m_stallCounter > 25) {
//      DriverStation.reportWarning("Intake motor might be stalling.", false);
//    }
//
//    SmartDashboard.putBoolean("Is Intake Deployed?: ", getIsIntakeDeployed());
//    SmartDashboard.put
//    SmartDashboard.putNumber("Blue", colorSensor.getColor().blue);
//    SmartDashboard.putNumber("Red", colorSensor.getColor().red);
//    SmartDashboard.putNumber("Green", colorSensor.getColor().green);
  }
}
