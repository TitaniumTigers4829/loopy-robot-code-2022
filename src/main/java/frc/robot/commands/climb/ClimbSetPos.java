// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbSetPos extends CommandBase {
  
  public int motorPos;
  private final ClimbSubsystem climbSubsystem;

  /** Creates a new ClimbSetPos. */
  public ClimbSetPos(ClimbSubsystem climbSubsystem, int motorPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climbSubsystem = climbSubsystem;
    this.motorPos = motorPos;
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbSubsystem.setPos(motorPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbSubsystem.setPos(motorPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Climb Pos Running", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumberArray("FInished encoder position", climbSubsystem.getBothPositions());
    return Math.abs(climbSubsystem.getPosition() - motorPos) <= 10;
  }
}
