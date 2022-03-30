// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class AcuatorTesting extends CommandBase {
  /** Creates a new FenderShot. */
  private final ShooterSubsystem shooter;
  private final DoubleSupplier pos;
  public AcuatorTesting(ShooterSubsystem shooter, DoubleSupplier pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.pos = pos;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double extension = ((pos.getAsDouble() + 1) / 2);
//    shooter.setHeight(extension);
    SmartDashboard.putNumber("Extension", extension);
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
