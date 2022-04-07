package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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

    // TODO: Tune all of the .withTimeouts

    addCommands(
        // 1. Backs up from the pad and intakes
        new ParallelCommandGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.firstPath),
            new IntakeWithTower(intakeSubsystem, towerSubsystem)
        ).withTimeout(1.8),

        // 2. Revs up the shooter while going in right in front of the third ball
        new ParallelRaceGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.secondPath).withTimeout(2.8),
            new AutoRevShoot(shooterSubsystem, LimelightSubsystem.getInstance()).withTimeout(4)
        ),

        // 3. Shoots the two balls it is currently holding then backs up and intakes the third ball
        new ParallelCommandGroup(
            new AutoShoot(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
                driveSubsystem, ledsSubsystem),
            new AutoDriveIntake(driveSubsystem, intakeSubsystem, towerSubsystem, 1.2)
        ).withTimeout(2),

        // 4. Shoots the third ball with the wheels still spinning from when the first two balls were shot
        new AutoShoot(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
            driveSubsystem, ledsSubsystem).withTimeout(1),
        // 5. Goes to the area where it can pick up cargo from human plays TODO: Make this into a ParallelCommandGroup
        new FollowTrajectory(driveSubsystem, PathWeaverConstants.thirdPath).withTimeout(3.7),
        // 6. Intakes long enough for the human players to load 2 balls
        new IntakeWithTower(intakeSubsystem, towerSubsystem).withTimeout(2),

        // 7. Revs up shooter while going closer to the hoop TODO: This path can be shortened if needed
        new ParallelRaceGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.fourthPath).withTimeout(3.7),
            new AutoRevShoot(shooterSubsystem, LimelightSubsystem.getInstance()).withTimeout(5)
        ),

        // 8. Shoots the two balls gotten from the human players
        new AutoShoot(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
            driveSubsystem, ledsSubsystem).withTimeout(2)
    );

  }

}
