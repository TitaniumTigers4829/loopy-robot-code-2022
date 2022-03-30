package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbSubsystem;

public class MidBarClimb extends SequentialCommandGroup {
   /**
     * Creates a new MidBarClimb.
    * step 1: (button 1) set climb hook above the second bar
    *   commands: ClimbVertical, ClimbFullExtension (MidBarLatchHooks)                                    CHECK
    * step 2: (button2) climb up // MID BAR ACHIEVED
    *   commands: ClimbBottomPositon                                                                       CHECK
    * step 3: (button3) Hooks up a little, arms back, arms all the way up, arms vertical again (solenoids)
    *   commands: ClimbSlightlyExtended, ClimbAngled, ClimbFullExtension, ClimbVertical (MidBarClimb)     CHECK
    * step 4: (button 2) climb up // HIGH BAR ACHIEVED
    * step 5: repeat steps 3 & 4 // TRAVERSAL ACHIEVED
     */
    public MidBarClimb(ClimbSubsystem climb) {
      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
//    Assumes we are under the first bar with the climb arm
//    all the way up
      addCommands(
          new ClimbSlightlyExtended(climb),
          new ClimbAngled(climb),
          new ClimbFullExtension(climb),
          new ClimbVertical(climb)
      );
    }
  }
