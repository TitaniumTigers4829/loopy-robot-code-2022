// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbHooksToSavedZero extends CommandBase {

  private final ClimbSubsystem climbSubsystem;

  /** Creates a new ClimbHooksToSavedZero. */
  public ClimbHooksToSavedZero(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    addRequirements(this.climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double leftHookHeight = climbSubsystem.getLeftClimbHookZeroValue();
    double rightHookHeight = climbSubsystem.getRightClimbHookZeroValue();

    if (climbSubsystem.getLeftHookHeight() > leftHookHeight && climbSubsystem.getIsLeftLimitSwitchPressed()) {
      climbSubsystem.setLeftMotorOutputManual(0);
    }
    else {
      climbSubsystem.setDesiredLeftHookHeight(leftHookHeight);
    }

    if (climbSubsystem.getLeftHookHeight() > rightHookHeight && climbSubsystem.getIsRightLimitSwitchPressed()) {
      climbSubsystem.setRightMotorOutputManual(0);
    }
    else {
      climbSubsystem.setDesiredRightHookHeight(rightHookHeight);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
