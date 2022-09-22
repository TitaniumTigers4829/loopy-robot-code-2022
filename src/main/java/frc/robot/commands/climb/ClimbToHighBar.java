package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbToHighBar extends SequentialCommandGroup {

  /**
   * Climbs to the high bar starting from the robot hanging on the mid bar with its telescoping arms
   * @param climbSubsystem
   */
  public ClimbToHighBar(ClimbSubsystem climbSubsystem) {
    addCommands(
        // Raises the hooks so that the static hooks are on the bar
        new ClimbSetPos(climbSubsystem, ClimbConstants.kSlightlyAboveHighBar + ClimbConstants.kClimbMaxPosConfirmationExtraHeight),
        // Sets the arms back
        new ClimbAngled(climbSubsystem),
        // Extends the arms to above the bar
        new ClimbSetPos(climbSubsystem, ClimbConstants.kClimbMaxHeight + ClimbConstants.kClimbMaxPosConfirmationExtraHeight),
        // Brings the arms back to the bar
        new WaitCommand(.5), // .3
        new ClimbVertical(climbSubsystem),
        // Waits a little bit
        new WaitCommand(1), // .8
        // Pulls the robot up to the next bar
        new ClimbSetPos(climbSubsystem, ClimbConstants.kSlightlyBelowBar - ClimbConstants.kClimbMaxPosConfirmationExtraHeight)
        // Static hooks click, but robot is still being held by the telescoping arms
    );
  }
}
