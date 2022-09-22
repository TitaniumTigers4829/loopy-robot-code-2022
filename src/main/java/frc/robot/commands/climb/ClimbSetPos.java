// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbSetPos extends CommandBase {
  
  public double motorPos;
  private final ClimbSubsystem climbSubsystem;

  /** Creates a new ClimbSetPos. 
   * @param motorPos: height in meters
  */
  public ClimbSetPos(ClimbSubsystem climbSubsystem, double motorPos) {
    this.climbSubsystem = climbSubsystem;
    this.motorPos = motorPos;
    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (climbSubsystem.getLeftHookHeight() > motorPos && climbSubsystem.getIsLeftLimitSwitchPressed()) {
      climbSubsystem.setLeftMotorOutputManual(0);
    }
    else {
      climbSubsystem.setDesiredLeftHookHeight(motorPos);
    }

    if (climbSubsystem.getRightHookHeight() > motorPos && climbSubsystem.getIsRightLimitSwitchPressed()) {
      climbSubsystem.setRightMotorOutputManual(0);
    }
    else {
      climbSubsystem.setDesiredRightHookHeight(motorPos);
    }
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.setLeftMotorOutputManual(0);
    climbSubsystem.setRightMotorOutputManual(0);
  }

  @Override
  public boolean isFinished() {
    return ((Math.abs(climbSubsystem.getLeftHookHeight() - motorPos) < .1 && Math.abs(climbSubsystem.getRightHookHeight() - motorPos) < .1)
            || ((climbSubsystem.getLeftHookHeight() > motorPos && climbSubsystem.getIsLeftLimitSwitchPressed()) && 
            (climbSubsystem.getRightHookHeight() > motorPos && climbSubsystem.getIsRightLimitSwitchPressed())));
  }
}
