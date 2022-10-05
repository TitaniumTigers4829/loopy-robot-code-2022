package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.PathWeaverConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.intake.IntakeWithTower;
import frc.robot.commands.tower.TowerIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

/**
 * This class is for the 5 Ball Auto Command
 */
public class FiveBallAutoCommand extends SequentialCommandGroup {

  public FiveBallAutoCommand(ShooterSubsystem shooterSubsystem, TowerSubsystem towerSubsystem,
      DriveSubsystem driveSubsystem, LEDsSubsystem ledsSubsystem, IntakeSubsystem intakeSubsystem) {

    addCommands(
        // 1. Backs up from the pad and intakes
        new ParallelCommandGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.firstPath5Ball, true),
            new IntakeWithTower(intakeSubsystem, towerSubsystem)
        ).withTimeout(1.4),

        // 2. Revs up the shooter while going in right in front of the third ball
        new ParallelRaceGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.secondPath5Ball,
                false).withTimeout(2.1), // 2.36
            new AutoRev(shooterSubsystem, LimelightSubsystem.getInstance(), ledsSubsystem),
            new TowerIntake(towerSubsystem)
        ),

        // 3. Shoots the two balls it is currently holding then intakes the third ball
        new AutoShootIntake(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
            driveSubsystem, ledsSubsystem, intakeSubsystem).withTimeout(3.45),

        // 4. Goes to the area where it can pick up cargo from human player while intaking
        new ParallelCommandGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.thirdPath5Ball, false),
            new IntakeWithTower(intakeSubsystem, towerSubsystem)
        ).withTimeout(3.6),

        // 5. Revs up shooter while going closer to the hoop
        new ParallelCommandGroup(
            new IntakeWithTower(intakeSubsystem, towerSubsystem),
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.fourthPath5Ball, false),
            new RunCommand(() -> shooterSubsystem.setShooterRPM(
                ShooterConstants.bottomMotorValues[2][1], // Sets the RPMs for 8.5 feet away
                ShooterConstants.topMotorValues[2][1]
            ))
        ).withTimeout(2.5),

        // 6. Sets the gyro offset
        new InstantCommand(() -> driveSubsystem.setGyroOffset(AutoConstants.fiveBallAutoOffset)),

        // 7. Shoots the two balls gotten from the human player and corner
        new AutoShoot(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
            driveSubsystem, ledsSubsystem)

    );

  }

}
