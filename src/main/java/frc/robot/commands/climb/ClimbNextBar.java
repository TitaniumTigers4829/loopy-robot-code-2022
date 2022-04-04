package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbNextBar extends SequentialCommandGroup {

  public ClimbNextBar(ClimbSubsystem climb){
    addCommands(
        // extends a little
        new ClimbSetPos(climb, ClimbConstants.kSlightlyExtended),
        // set solenoid extended
        new ClimbAngled(climb),
        // wait a sec
        new RunCommand(()-> Timer.delay(1.5)),
        // extend
        new ClimbSetPos(climb, ClimbConstants.kClimbMaxHeight),
        // set solenoid retracted
        new ClimbVertical(climb)
        // Retract climb
//        new ClimbSetPos(climb, 0)
        // should be done with this bar
        // static hooks click
    );
  }
}
