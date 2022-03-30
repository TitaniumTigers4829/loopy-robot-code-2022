// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidBarLatchHooks extends SequentialCommandGroup {
  /**
   * Creates a new MidBarLatchHooks.
   */
  public MidBarLatchHooks(ClimbSubsystem climb) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
//    Assumes we are under the first bar with the climb arm
//    all the way up
    addCommands(
        new ClimbVertical(climb),
        new ClimbFullExtension(climb)
    );
  }
}
