package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbToTraversalBar extends SequentialCommandGroup {

  /**
   * Climbs to the traversal bar starting from the robot hanging on the high bar with its telescoping arms
   * @param climbSubsystem
   */
  public ClimbToTraversalBar(ClimbSubsystem climbSubsystem) {
    addCommands(
        // Raises the hooks so that the static hooks are on the bar
        new ClimbSetPos(climbSubsystem, ClimbConstants.kSlightlyAboveBar + ClimbConstants.kClimbMaxPosConfirmationExtraHeight),
        // Sets the arms back
        new ClimbAngled(climbSubsystem),
        // Extends the arms to above the bar
        new ClimbSetPos(climbSubsystem, ClimbConstants.kClimbMaxHeight + ClimbConstants.kClimbMaxPosConfirmationExtraHeight),
        // Brings the arms back to the bar
        // new WaitCommand(.2),
        new ClimbVertical(climbSubsystem),
        // Waits a little bit
        new WaitCommand(.2),
        // Pulls the robot up to the traversal bar
        new ClimbSetPos(climbSubsystem, ClimbConstants.kTraversalBarFinalHeight - ClimbConstants.kClimbMaxPosConfirmationExtraHeight)
        // Ends with the robot hanging on the traversal bar with the telescoping arms, doesn't use static hooks
    );
  }
}
