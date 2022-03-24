// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SetFlyWheelSpeed extends CommandBase {

  private ShooterSubsystem shooterSubsystem;
  private final double speed;

  /** Creates a new SetFlyWheelSpeed. */
  public SetFlyWheelSpeed(ShooterSubsystem shooterSubsystem, double speed) {
    this.shooterSubsystem = shooterSubsystem;
    this.speed = speed;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setSpeed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
