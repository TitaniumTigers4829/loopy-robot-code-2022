package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PathWeaverConstants;
import frc.robot.commands.intake.EjectCommand;
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
public class TwoBallAutoCommand extends SequentialCommandGroup {

  public TwoBallAutoCommand(ShooterSubsystem shooterSubsystem, TowerSubsystem towerSubsystem,
      DriveSubsystem driveSubsystem, LEDsSubsystem ledsSubsystem, IntakeSubsystem intakeSubsystem) {

    addCommands(
        // 1. Backs up from the pad and intakes the second ball
        new ParallelCommandGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.firstPath2Ball),
            new IntakeWithTower(intakeSubsystem, towerSubsystem)
        ).withTimeout(2),

        // 2. Shoots two balls
        new AutoShoot(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
            driveSubsystem, ledsSubsystem).withTimeout(4),

        // 3. Picks up an enemy ball, then goes into the hangar
        new ParallelCommandGroup(
            new FollowTrajectory(driveSubsystem, PathWeaverConstants.secondPath2Ball).withTimeout(3.3),
            new IntakeWithTower(intakeSubsystem, towerSubsystem).withTimeout(1.8)
        ),

        // 4. Ejects the enemy ball
        new EjectCommand(towerSubsystem).withTimeout(1.5),

        // 5. Goes near white line in the direction of a ball and the direction to reset the gyro at
        new FollowTrajectory(driveSubsystem, PathWeaverConstants.thirdPath2Ball).withTimeout(3)
    );

  }

}
