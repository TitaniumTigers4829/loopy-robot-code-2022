// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectoryPathPlanner extends CommandBase {

  private DriveSubsystem driveSubsystem;
  private String filePath;

  PPSwerveControllerCommand followTrajectoryPathPlannerCommand;
  private boolean done = false;

  /** Creates a new FollowTrajectoryPathPlanner. */
  public FollowTrajectoryPathPlanner(DriveSubsystem driveSubsystem, String filePath) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    
    this.filePath = filePath;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Makes a trajectory                                                     Vel  Accel
    PathPlannerTrajectory trajectoryToFollow = PathPlanner.loadPath(filePath, 4.5, 3.25);

    // PID controllers
    PIDController xController = new PIDController(PathPlannerConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(PathPlannerConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
    PathPlannerConstants.kPThetaController, 0, 0, PathPlannerConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI); // Makes it so wheels don't have to turn more than 90 degrees

    // Create a PPSwerveControllerCommand. This is almost identical to WPILib's SwerveControllerCommand, but it uses the holonomic rotation
    // from the PathPlannerTrajectory to control the robot's rotation.
    // See the WPILib SwerveControllerCommand for more info on what you need to pass to the command
    followTrajectoryPathPlannerCommand = new PPSwerveControllerCommand(
      trajectoryToFollow,
      driveSubsystem::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,
      xController,
      yController,
      thetaController,
      driveSubsystem::setModuleStates,
      driveSubsystem
    );
    
    followTrajectoryPathPlannerCommand.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    done = followTrajectoryPathPlannerCommand.isFinished();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
