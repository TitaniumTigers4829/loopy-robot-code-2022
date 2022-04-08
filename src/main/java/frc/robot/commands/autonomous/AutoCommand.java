package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PathWeaverConstants;
import frc.robot.commands.intake.IntakeWithTower;
import frc.robot.commands.tower.TowerIntake;
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
        ).withTimeout(1.4),

        // 2. Revs up the shooter while going in right in front of the third ball
        new ParallelRaceGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.secondPath).withTimeout(1.8),
            new AutoRevShoot(shooterSubsystem, LimelightSubsystem.getInstance()),
            new TowerIntake(towerSubsystem)
        ),

        // 3. Shoots the two balls it is currently holding, then intakes the third and shoots it
        new AutoShootIntake(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
            driveSubsystem, ledsSubsystem, intakeSubsystem).withTimeout(4.3),

        // 4. Intakes while going to pick up the fourth ball and going to get the fifth ball
        new ParallelCommandGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.thirdPath),
            new IntakeWithTower(intakeSubsystem, towerSubsystem)
        ).withTimeout(2.5),

        // 5. Revs up shooter and intakes while going back towards the hoop
        new ParallelCommandGroup(
            new IntakeWithTower(intakeSubsystem, towerSubsystem).withTimeout(2),
            new ParallelRaceGroup(
                new FollowTrajectory(driveSubsystem, PathWeaverConstants.fourthPath).withTimeout(
                    2.7),
                new AutoRevShoot(shooterSubsystem, LimelightSubsystem.getInstance())
            )
        ),

        // 6. Shoots the two balls gotten
        new AutoShoot(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
            driveSubsystem, ledsSubsystem).withTimeout(4)
    );

  }

}
