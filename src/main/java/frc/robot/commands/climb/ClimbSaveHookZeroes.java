// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbSaveHookZeroes extends CommandBase {

  private final ClimbSubsystem climbSubsystem;

  /** Creates a new ClimbSaveArmZeroes. */
  public ClimbSaveHookZeroes(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    addRequirements(this.climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    double leftHookHeight = climbSubsystem.getLeftHookHeight();
    double rightHookHeight = climbSubsystem.getRightHookHeight();

    SaveClimbZeroes.writeClimbZeroes(leftHookHeight, rightHookHeight);
  
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
    return false;
  }
}
