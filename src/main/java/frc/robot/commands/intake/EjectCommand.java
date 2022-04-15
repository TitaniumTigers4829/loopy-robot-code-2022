package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class EjectCommand extends CommandBase {

  private final TowerSubsystem m_towerSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;

  /**
   * Ejects the bottom ball. By default, a "while held" command.
   *
   * @param m_towerSubsystem The tower subsystem used by this command.
   */
  public EjectCommand(TowerSubsystem m_towerSubsystem, IntakeSubsystem m_intakeSubsystem) {
    this.m_towerSubsystem = m_towerSubsystem;
    this.m_intakeSubsystem = m_intakeSubsystem;
    addRequirements(m_towerSubsystem, m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    m_towerSubsystem.setTowerMotorsSpeed(-TowerConstants.towerMotorSpeed);
    m_intakeSubsystem.setSolenoidDeployed();
    m_intakeSubsystem.setMotorFullPowerOut();
  }

  @Override
  public void execute() {
    m_towerSubsystem.setTowerMotorsSpeed(-TowerConstants.towerMotorSpeed);
    m_intakeSubsystem.setMotorFullPowerOut();
  }

  @Override
  public void end(boolean interrupted) {
    m_towerSubsystem.setTowerMotorsSpeed(0);
    m_intakeSubsystem.setMotorStopped();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
