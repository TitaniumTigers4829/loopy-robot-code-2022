// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    
    addCommands(
      // Starts assuming the telescoping arms are slightly above the first bar
      new InstantCommand(climbSubsystem::resetEncoders),
      new ClimbSetPos(climbSubsystem, ClimbConstants.kSlightlyBelowBar - ClimbConstants.kClimbMaxPosConfirmationExtraHeight),
      new WaitCommand(.1),
      new ClimbToHighBar(climbSubsystem),
      new WaitCommand(.5),
      new ClimbToTraversalBar(climbSubsystem),
      new WaitCommand(50)
    );
  }
}
