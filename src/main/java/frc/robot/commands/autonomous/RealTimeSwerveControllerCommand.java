// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PiSubsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link Trajectory} with a swerve drive.
 *
 * <p>This command outputs the raw desired Swerve Module States ({@link SwerveModuleState}) in an
 * array. The desired wheel and module rotation velocities should be taken from those and used in
 * velocity PIDs.
 *
 * <p>The robot angle controller does not follow the angle given by the trajectory but rather goes
 * to the angle given in the final state of the trajectory.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
@SuppressWarnings("MemberName")
public class RealTimeSwerveControllerCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final PiSubsystem m_piSubsystem;
  private final Timer m_timer = new Timer();
  private final Supplier<Pose2d> m_pose;
  private final SwerveDriveKinematics m_kinematics;
  private final HolonomicDriveController m_controller;
  private final Consumer<SwerveModuleState[]> m_outputModuleStates;

  private Supplier<Rotation2d> m_desiredRotation;
  private Trajectory m_trajectory;

  /**
   * Constructs a new SwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   *    
   * @param driveSubsystem The subsystem for the swerve drive.
   * @param piSubsystem The subsystem for the pi.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param desiredRotation The angle that the drivetrain should be facing. This is sampled at each
   *     time step.
   * @param outputModuleStates The raw output module states from the position controllers.
   */
  @SuppressWarnings("ParameterName")
  public RealTimeSwerveControllerCommand(
      DriveSubsystem driveSubsystem,
      PiSubsystem piSubsystem,
      Supplier<Pose2d> pose,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Consumer<SwerveModuleState[]> outputModuleStates) {
    m_driveSubsystem = driveSubsystem;
    m_piSubsystem = piSubsystem;
    m_pose = requireNonNullParam(pose, "pose", "SwerveControllerCommand");
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveControllerCommand");

    m_controller =
        new HolonomicDriveController(
            requireNonNullParam(xController, "xController", "SwerveControllerCommand"),
            requireNonNullParam(yController, "yController", "SwerveControllerCommand"),
            requireNonNullParam(thetaController, "thetaController", "SwerveControllerCommand"));

    m_outputModuleStates =
        requireNonNullParam(outputModuleStates, "outputModuleStates", "SwerveControllerCommand");

    addRequirements(driveSubsystem, piSubsystem);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();

    m_driveSubsystem.zeroHeading();
    m_driveSubsystem.resetOdometry(new Pose2d());

    if (m_piSubsystem.cargoInView()) {
      double cargoDistance = m_piSubsystem.getCargoDistance();
      double x = m_piSubsystem.getCargoXPos();
      double y = m_piSubsystem.getCargoYPos(x, cargoDistance);
      double theta = m_piSubsystem.getCargoTheta(x, cargoDistance);
      m_trajectory = m_piSubsystem.generateTrajectory(x, y, theta);
    } else {
      m_trajectory = m_piSubsystem.generateTrajectory(.01, .01, 0);
    }

    m_desiredRotation = () -> m_trajectory.getStates().get(m_trajectory.getStates().size() - 1).poseMeters.getRotation();
    m_driveSubsystem.resetOdometry(m_trajectory.getInitialPose());
  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void execute() {
    double curTime = m_timer.get();
    var desiredState = m_trajectory.sample(curTime);

    var targetChassisSpeeds =
        m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());
    var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

    m_outputModuleStates.accept(targetModuleStates);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    // return false;
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds()); // TODO: This probably has to be changed
  }
}
