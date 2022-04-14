package frc.robot.commands.autonomous;

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
 * This class is for the auto command
 */
public class AutoCommand extends SequentialCommandGroup {

  public AutoCommand(ShooterSubsystem shooterSubsystem, TowerSubsystem towerSubsystem,
      DriveSubsystem driveSubsystem, LEDsSubsystem ledsSubsystem, IntakeSubsystem intakeSubsystem) {

    addCommands(
        // 1. Backs up from the pad and intakes
        new ParallelCommandGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.firstPath),
            new IntakeWithTower(intakeSubsystem, towerSubsystem)
        ).withTimeout(1.4),

        // 2. Revs up the shooter while going in right in front of the third ball
        new ParallelRaceGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.secondPath).withTimeout(2.36),
            new AutoRevAndAim(shooterSubsystem, LimelightSubsystem.getInstance(), ledsSubsystem),
            new TowerIntake(towerSubsystem)
        ),

        // 3. Shoots the two balls it is currently holding then intakes the third ball
        new AutoShootIntake(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
            driveSubsystem, ledsSubsystem, intakeSubsystem).withTimeout(3.5),

        // 5. Goes to the area where it can pick up cargo from human plays
        new ParallelCommandGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.thirdPath),
            // 6. Intakes long enough for the human players to load 2 balls
            new IntakeWithTower(intakeSubsystem, towerSubsystem)
        ).withTimeout(3.6),

        // 7. Revs up shooter while going closer to the hoop
        new ParallelCommandGroup(
            new IntakeWithTower(intakeSubsystem, towerSubsystem).withTimeout(2),
            new ParallelRaceGroup(
                new FollowTrajectory(driveSubsystem, PathWeaverConstants.fourthPath).withTimeout(
                    2.4),
                new AutoRevAndAim(shooterSubsystem, LimelightSubsystem.getInstance(), ledsSubsystem)
            )
        ),

        // 8. Shoots the two balls gotten from the human players
        new AutoShoot(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
            driveSubsystem, ledsSubsystem).withTimeout(4)
    );

  }

}
