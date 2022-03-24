package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class IntakeActiveTeleop extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intakeSubsystem;
  private final TowerSubsystem m_towerSubsystem;

  /**
   * Creates IntakeActiveTeleop. By default, a "while held" command.
   *
   * @param m_intakeSubsystem The climb subsystem used by this command.
   * @param m_towerSubsystem The tower subsystem used by this command.
   */
  public IntakeActiveTeleop(IntakeSubsystem m_intakeSubsystem, TowerSubsystem towerSubsystem) {
    this.m_intakeSubsystem = m_intakeSubsystem;
    this.m_towerSubsystem = towerSubsystem;
    addRequirements(m_intakeSubsystem, towerSubsystem);
  }

  @Override
  public void initialize() {
    m_intakeSubsystem.setSolenoidDeployed();
    m_intakeSubsystem.setMotorFullPowerIn();
  }

  @Override
  public void execute() {
    m_intakeSubsystem.setMotorFullPowerIn();

    // Code to load balls
    if (!m_towerSubsystem.getTopBallIn()) {
      m_towerSubsystem.setTopTowerMotorSpeed(TowerConstants.towerMotorLoadSpeed);
      m_towerSubsystem.setBottomTowerMotorSpeed(TowerConstants.towerMotorLoadSpeed);
    } else {
      m_towerSubsystem.setTopTowerMotorSpeed(0);
      if (!m_towerSubsystem.getBottomBallIn()) {
        m_towerSubsystem.setBottomTowerMotorSpeed(TowerConstants.towerMotorLoadSpeed);
      } else {
        m_towerSubsystem.setBottomTowerMotorSpeed(0);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setMotorStopped();
    m_intakeSubsystem.setSolenoidRetracted();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
