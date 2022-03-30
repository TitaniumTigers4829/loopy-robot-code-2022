// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitCommand extends CommandBase {

  private final double secondsToWait;

  private Timer timer = new Timer();
  private double startTime = timer.get();
  
  /** Creates a new WaitCommand. */
  public WaitCommand(double secondToWait) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.secondsToWait = secondToWait;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= startTime + secondsToWait);
  }
}
