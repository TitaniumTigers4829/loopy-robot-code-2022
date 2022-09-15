package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbNextBar extends SequentialCommandGroup {

  public ClimbNextBar(ClimbSubsystem climb){
    addCommands(
        // Raises the hooks so that the static hooks are on the bar
        new ClimbSetPos(climb, ClimbConstants.kSlightlyAboveBar + ClimbConstants.kClimbMaxPosConfirmationExtraHeight),
        // Waits a little bit
        // new WaitCommand(1),
        // Sets the arms back
        new ClimbAngled(climb),
        // Waits a little bit
        // new WaitCommand(1),
        // Extends the arms to above the bar
        new ClimbSetPos(climb, ClimbConstants.kClimbMaxHeight + ClimbConstants.kClimbMaxPosConfirmationExtraHeight),
        // Brings the arms back to the bar
        new WaitCommand(1),
        new ClimbVertical(climb),
         // Waits a little bit
        new WaitCommand(.8),
        // Pulls the robot up to the next bar
        new ClimbSetPos(climb, ClimbConstants.kSlightlyBelowBar - ClimbConstants.kClimbMaxPosConfirmationExtraHeight)
        // Static hooks click, done with this bar
    );
  }
}
