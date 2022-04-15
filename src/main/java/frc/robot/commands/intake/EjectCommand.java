package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class EjectCommand extends CommandBase {

  private final TowerSubsystem m_towerSubsystem;

  /**
   * Ejects the bottom ball. By default, a "while held" command.
   *
   * @param m_towerSubsystem The tower subsystem used by this command.
   */
  public EjectCommand(TowerSubsystem m_towerSubsystem) {
    this.m_towerSubsystem = m_towerSubsystem;
    addRequirements(m_towerSubsystem);
  }

  @Override
  public void initialize() {
    m_towerSubsystem.setTowerMotorsSpeed(-TowerConstants.towerMotorSpeed);
  }

  @Override
  public void execute() {
    m_towerSubsystem.setTowerMotorsSpeed(-TowerConstants.towerMotorSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_towerSubsystem.setTowerMotorsSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
