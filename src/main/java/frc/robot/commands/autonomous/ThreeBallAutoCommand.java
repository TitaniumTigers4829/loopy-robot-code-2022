package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PathWeaverConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.intake.IntakeWithTower;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

/**
 * This class is for the 3 Ball Auto Command
 */
public class ThreeBallAutoCommand extends SequentialCommandGroup {

  public ThreeBallAutoCommand(ShooterSubsystem shooterSubsystem, TowerSubsystem towerSubsystem,
      DriveSubsystem driveSubsystem, LEDsSubsystem ledsSubsystem, IntakeSubsystem intakeSubsystem) {

    addCommands(
        // 1. Crosses the line while running the intake backwards
        new ParallelCommandGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.firstPath3Ball, true),
            new AutoIntakeBlow(intakeSubsystem)
        ).withTimeout(1.4),

        // 2. Runs the intake backwards to blow away the ball
        new AutoIntakeBlow(intakeSubsystem).withTimeout(1.5),

        // 3. Slowly moves towards the ball while running the intake backwards
        new ParallelCommandGroup(
            new SetDriveSpeed(driveSubsystem, .15, 0), // TODO: Tune Values
            new AutoIntakeBlow(intakeSubsystem)
        ).withTimeout(2),

        new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, false)), // Stops the robot

        // 4. Gives the intake a little time to start spinning the other way
        new IntakeWithTower(intakeSubsystem, towerSubsystem).withTimeout(1),

        // 5. Slowly moves towards the ball while intaking
        new AutoDriveIntake(driveSubsystem, intakeSubsystem, towerSubsystem, .622).withTimeout(1), // TODO: Tune Values

        // 6. Goes back towards the hoop while bumping an enemy ball and revving up the shooter
        new ParallelCommandGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.secondPath3Ball, false),
            new RunCommand(() -> shooterSubsystem.setShooterRPM(
                ShooterConstants.bottomMotorValues[0][1], // Sets the RPMs for 5.5 feet away
                ShooterConstants.topMotorValues[0][1]
            ))
        ).withTimeout(2.7),

        // 7. Shoots two balls
        new AutoShoot(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
            driveSubsystem, ledsSubsystem).withTimeout(1.9),

        // 8. Picks up the third ball
        new ParallelCommandGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.thirdPath3Ball, false),
            new IntakeWithTower(intakeSubsystem, towerSubsystem)
        ).withTimeout(1.4),

        // 9. Shoots the third ball
        new AutoShoot(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
            driveSubsystem, ledsSubsystem)
    );

  }

}
