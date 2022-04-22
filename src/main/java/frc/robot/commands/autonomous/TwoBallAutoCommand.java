package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PathWeaverConstants;
import frc.robot.commands.intake.EjectCommand;
import frc.robot.commands.intake.IntakeWithTower;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

/**
 * This class is for the 5 Ball Auto Command
 */
public class TwoBallAutoCommand extends SequentialCommandGroup {

  public TwoBallAutoCommand(ShooterSubsystem shooterSubsystem, TowerSubsystem towerSubsystem,
      DriveSubsystem driveSubsystem, LEDsSubsystem ledsSubsystem, IntakeSubsystem intakeSubsystem) {

    addCommands(
        // 1. Backs up from the pad and intakes the second ball
        new ParallelCommandGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.firstPath2Ball, true),
            new IntakeWithTower(intakeSubsystem, towerSubsystem)
        ).withTimeout(2),

        // 2. Moves closer to the hoop
        new ParallelCommandGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.secondPath2Ball, false),
            new IntakeWithTower(intakeSubsystem, towerSubsystem),
            new AutoRev(shooterSubsystem, LimelightSubsystem.getInstance(), ledsSubsystem)
        ).withTimeout(1.2),

        // 3. Shoots two balls
        new AutoShoot(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
            driveSubsystem, ledsSubsystem).withTimeout(3.6),

        // 4. Picks up an enemy ball, then goes into the hangar
        new ParallelCommandGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.thirdPath2Ball, false).withTimeout(3.61),
            new IntakeWithTower(intakeSubsystem, towerSubsystem).withTimeout(2.4)
        ),

        // 5. Waits so when the ball is ejected it doesn't have any sideways momentum
        new WaitCommand(.3),

        // 6. Ejects the enemy ball
        new EjectCommand(towerSubsystem, intakeSubsystem).withTimeout(1.6),

        // 7. Goes near white line in the direction of a ball and the direction to reset the gyro at
        new FollowTrajectory(driveSubsystem, PathWeaverConstants.fourthPath2Ball, false).withTimeout(2.5),

        new RunCommand(
            ()-> driveSubsystem.drive(0, 0, 0, false)
        ).withTimeout(0.1),

        new InstantCommand(driveSubsystem::zeroHeading)
    );

  }

}
