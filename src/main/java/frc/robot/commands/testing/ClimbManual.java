package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ClimbManual extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbSubsystem m_climbSubsystem;

  private final DoubleSupplier m_leftStick, m_rightStick;
  private final boolean m_doPaired, m_doSolenoids;
  private final BooleanSupplier m_setVertical, m_setAngled;

  /**
   * Creates ClimbManual.
   *
   * @param subsystem The climb subsystem used by this command.
   * @param doPaired Whether we should control the climb arms together, will be determined by
   *                 whether rb was being pressed when b button was pressed.
   *
   * @param doSolenoids Whether we want to control the solenoids, will be determined by whether lb
   *                    was being pressed when the b button was pressed.
   */
  public ClimbManual(ClimbSubsystem subsystem, DoubleSupplier leftStick, DoubleSupplier rightStick,
      boolean doPaired, BooleanSupplier vertical, BooleanSupplier angled, boolean doSolenoids) {
    m_climbSubsystem = subsystem;
    addRequirements(m_climbSubsystem);  // Use addRequirements() to declare subsystem dependencies.

    m_leftStick = leftStick;
    m_rightStick = rightStick;
    m_doPaired = doPaired;
    m_setVertical = vertical;
    m_setAngled = angled;
    m_doSolenoids = doSolenoids;
  }

  @Override
  public void initialize() {
    if (m_doPaired && !m_doSolenoids) {
      new ClimbManualPairedControl(m_climbSubsystem, m_rightStick);
    }
    else if (m_doSolenoids && !m_doPaired){
      new ClimbManualSolenoidControl(m_climbSubsystem, m_setVertical, m_setAngled);
    } else {
      new ClimbManualIndependentControl(m_climbSubsystem, m_leftStick, m_rightStick);
    }
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.getCurrentCommand().end(true);
    m_climbSubsystem.setLeftMotorOutputManual(0); // just to be safe, should have already been done
    m_climbSubsystem.setRightMotorOutputManual(0); // just to be safe, should have already been done
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

