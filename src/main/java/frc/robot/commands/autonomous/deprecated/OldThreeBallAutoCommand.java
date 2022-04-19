package frc.robot.commands.autonomous.deprecated;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PathWeaverConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.autonomous.AutoShoot;
import frc.robot.commands.autonomous.FollowTrajectory;
import frc.robot.commands.intake.IntakeWithTower;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

/**
 * This class is for the Old 3 Ball Auto Command
 */
public class OldThreeBallAutoCommand extends SequentialCommandGroup {

  public OldThreeBallAutoCommand(ShooterSubsystem shooterSubsystem, TowerSubsystem towerSubsystem,
      DriveSubsystem driveSubsystem, LEDsSubsystem ledsSubsystem, IntakeSubsystem intakeSubsystem) {

    addCommands(
        // 1. Goes in front of ball across the line
        new FollowTrajectory(driveSubsystem, PathWeaverConstants.firstPathOld3Ball, true).withTimeout(
            1.8),

        // 2. Intakes for a while to suck the ball in
        new IntakeWithTower(intakeSubsystem, towerSubsystem).withTimeout(5),

        // 3. Goes back towards the hoop while bumping an enemy ball and revving up the shooter
        new ParallelCommandGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.secondPathOld3Ball, false),
            new RunCommand(() -> shooterSubsystem.setShooterRPM(
                ShooterConstants.bottomMotorValues[0][1], // Sets the RPMs for 5.5 feet away
                ShooterConstants.topMotorValues[0][1]
            ))
        ).withTimeout(2.9),

        // 4. Shoots two balls
        new AutoShoot(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
            driveSubsystem, ledsSubsystem).withTimeout(1.9),

        // 5. Goes to pick up the third ball
        new ParallelCommandGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.thirdPathOld3Ball, false),
            new IntakeWithTower(intakeSubsystem, towerSubsystem)
        ).withTimeout(1.3),

        // 6. Goes closer to the tower while reving up the shooter
        new ParallelCommandGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.fourthPathOld3Ball, false),
            new RunCommand(() -> shooterSubsystem.setShooterRPM(
                ShooterConstants.bottomMotorValues[0][1], // Sets the RPMs for 5.5 feet away
                ShooterConstants.topMotorValues[0][1]
            ))
        ).withTimeout(1.2),

        // 7. Shoots the third ball
        new AutoShoot(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
            driveSubsystem, ledsSubsystem)
    );

  }

}
