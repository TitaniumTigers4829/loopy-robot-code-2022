// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import java.util.function.BooleanSupplier;

public class ClimbWithButtons extends CommandBase {

  /**
   * Creates a new ClimbWithButtons.
   */
  private final ClimbSubsystem climb;
  //  scuffed
  private final BooleanSupplier Lup;
  private final BooleanSupplier Ldown;
  private final BooleanSupplier Rup;
  private final BooleanSupplier Rdown;
  private final BooleanSupplier Armvert;
  private final BooleanSupplier Armdown;


  public ClimbWithButtons(ClimbSubsystem climb, BooleanSupplier Lup, BooleanSupplier Ldown,
      BooleanSupplier Rup, BooleanSupplier Rdown, BooleanSupplier Armvert, BooleanSupplier Armdown) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climb = climb;
    this.Lup = Lup;
    this.Ldown = Ldown;
    this.Rup = Rup;
    this.Rdown = Rdown;
    this.Armvert = Armvert;
    this.Armdown = Armdown;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Manually controlling climb arm height
    if (Lup.getAsBoolean()) {
      climb.setLeftMotorOutputManual(0.75);
    } else if ((Ldown.getAsBoolean()) && (!climb.getIsLeftLimitSwitchPressed())) {
      climb.setLeftMotorOutputManual(-0.75);
    } else {
      climb.setLeftMotorOutputManual(0);
    }
    if (Rup.getAsBoolean()) {
      climb.setRightMotorOutputManual(0.75);
    } else if ((Rdown.getAsBoolean()) && (!climb.getIsRightLimitSwitchPressed())) {
      climb.setRightMotorOutputManual(-0.75);
    } else {
      climb.setRightMotorOutputManual(0);
    }

    // Manually controlling climb arm angle
    if (Armvert.getAsBoolean()) {
      climb.setClimbVertical();
    } else if (Armdown.getAsBoolean()) {
      climb.setClimbAngled();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.setLeftMotorOutputManual(0);
    climb.setRightMotorOutputManual(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
