// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbCommand extends SequentialCommandGroup {

  /**
   * Command to climb
   */
  public ClimbCommand(ClimbSubsystem climbSubsystem) {
    // Starts assuming the telescoping arms are slightly above the first bar
    addCommands(
        // Reel into the bar
        new ClimbSetPos(climbSubsystem, 0),
        // should be done with bar 1
        // static hooks click
        // ----------------------------------------------------------------------------
        // extends a little
        new ClimbSetPos(climbSubsystem, ClimbConstants.kSlightlyExtended),
        // set solenoid extended
        new ClimbAngled(climbSubsystem),
        // extend
        new ClimbSetPos(climbSubsystem, ClimbConstants.kSecondBarPos),
        // set solenoid retracted
        new ClimbVertical(climbSubsystem),
        // Retract climb
        new ClimbSetPos(climbSubsystem, 0),
        // should be done with bar 2
        // static hooks click
        // ----------------------------------------------------------------------------
        // extends a little
        new ClimbSetPos(climbSubsystem, ClimbConstants.kSlightlyExtended),
        // set solenoid extended
        new ClimbAngled(climbSubsystem),
        // extend
        new ClimbSetPos(climbSubsystem, ClimbConstants.kSecondBarPos),
        // set solenoid retracted
        new ClimbVertical(climbSubsystem),
        // Retract climb
        new ClimbSetPos(climbSubsystem, 0)
        // should be done with bar 3
        // yay we finished climb
    );
  }
}
