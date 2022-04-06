package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PathWeaverConstants;
import frc.robot.commands.intake.IntakeWithTower;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

/**
 * This class is for the auto command for if the robot is blue TODO: Check if it will work with red,
 * it probably will
 */
public class AutoCommand extends SequentialCommandGroup {

  public AutoCommand(ShooterSubsystem shooterSubsystem, TowerSubsystem towerSubsystem,
      DriveSubsystem driveSubsystem, LEDsSubsystem ledsSubsystem, IntakeSubsystem intakeSubsystem) {

    addCommands( // FIXME: All of the numbers need to be tuned
        // 1. Backs up from the pad
        new FollowTrajectory(driveSubsystem, PathWeaverConstants.firstPath).withTimeout(3),
        // Will finish the path then the next command is called
        // 2. Intakes the ball behind it
        new AutoDriveIntake(driveSubsystem, intakeSubsystem, towerSubsystem, 2).withTimeout(1.5),
        // 3. Goes to right in front of the third ball
        new FollowTrajectory(driveSubsystem, PathWeaverConstants.secondPath).withTimeout(3),

        // 4. Shoots the two balls it is currently holding
        new AutoShoot(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
            driveSubsystem, ledsSubsystem).withTimeout(3),
        // 5. Backs up and intakes the third ball
        new AutoDriveIntake(driveSubsystem, intakeSubsystem, towerSubsystem, 1.2).withTimeout(1.2),

        // 6. Shoots the third ball with the wheels still spinning from when the first two balls were shot
        new AutoShoot(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
            driveSubsystem, ledsSubsystem).withTimeout(3),
        // 7. Goes to the area where it can pick up cargo from human plays
        new FollowTrajectory(driveSubsystem, PathWeaverConstants.thirdPath).withTimeout(4),
        // 8. Intakes long enough for the human players to load 2 balls
        new IntakeWithTower(intakeSubsystem, towerSubsystem).withTimeout(2),
        // 9. Drives closer to the hoop
        new FollowTrajectory(driveSubsystem, PathWeaverConstants.fourthPath).withTimeout(3),
        // 10. Shoots the two balls gotten from the human players
        new AutoShoot(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
            driveSubsystem, ledsSubsystem).withTimeout(2)
        );

  }

}
