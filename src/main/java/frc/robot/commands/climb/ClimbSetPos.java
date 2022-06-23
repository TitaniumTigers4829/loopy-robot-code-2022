// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbSetPos extends CommandBase {
  
  public double motorPos;
  private final ClimbSubsystem climbSubsystem;

  /** Creates a new ClimbSetPos. */
  public ClimbSetPos(ClimbSubsystem climbSubsystem, double motorPos) {
    this.climbSubsystem = climbSubsystem;
    this.motorPos = motorPos;
    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Climb Pos Running", true);
  }

  @Override
  public void execute() {
    climbSubsystem.setDesiredLeftHookHeight(motorPos);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Climb Pos Running", false);
    climbSubsystem.setLeftMotorOutputManual(0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(climbSubsystem.getLeftEncoderValue() - motorPos) <= 10;
  }
}
