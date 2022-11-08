// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AIRobotConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class PiSubsystem extends SubsystemBase {

  private SwerveDriveOdometry m_odometry;
  private Gyro m_gyro;
  private double cargoPixelHeight = -1;
  private double cargoPixelXOffset = -1;

  // private static final Gyro m_gyro = new AHRS(SPI.Port.kMXP);
  // private static final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
  //     m_gyro.getRotation2d());

  /** Creates a new Jetson object */
  public PiSubsystem(SwerveDriveOdometry m_odometry, Gyro m_gyro) {
    this.m_odometry = m_odometry;
    this.m_gyro = m_gyro;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    cargoPixelHeight = SmartDashboard.getNumber(AIRobotConstants.cargoPixelHeightKey, -1);
    cargoPixelXOffset = SmartDashboard.getNumber(AIRobotConstants.cargoPixelXOffsetKey, -1);
    SmartDashboard.putNumber("error s", m_odometry.getPoseMeters().getRotation().getDegrees() + m_gyro.getAngle());
  }

  /**
   * Generates a trajectory for the robot to follow based on coordinates relative
   * to the robot.
   * 
   * @param x        the x offset in meters relative to the robot.
   * @param y        the y offset in meters relative to the robot.
   * @param rotation the angle in degrees of the offset of the ball from the
   *                 center of the camera.
   * @return
   */
  public Trajectory generateTrajectory() {

    // double x = 0.01;
    // double y = 0.01;
    // double rotation = 0;

    double x = 0.01;
    double y = 2.5;
    double rotation = 0;

    // if (cargoPixelHeight != -1) {
    //   x = getCargoXPos(cargoPixelHeight, cargoPixelXOffset);
    //   y = getCargoYPos(x, getCargoDistance(cargoPixelHeight));
    //   rotation = 0;
    // }

    double curX = m_odometry.getPoseMeters().getX();
    double curY = m_odometry.getPoseMeters().getY();

    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            // .setKinematics(SwerveDriveConstants.kDriveKinematics)
            .setStartVelocity(0)
            .setEndVelocity(0);

    // This accounts for the gyro and odometry returning different thetas
    double odometryDegreeOffset = m_odometry.getPoseMeters().getRotation().getDegrees() + m_gyro.getAngle();
    
    // TODO: BET $20 over whether initial pose needs to be current pos or 0
    // Lori gets $20 if initial pose is current pos, Jack gets $20 if otherwise
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(curY, curX, Rotation2d.fromDegrees(odometryDegreeOffset)),
        List.of(),
        new Pose2d(curY + y, curX + x, Rotation2d.fromDegrees(odometryDegreeOffset + rotation)),
        config
    );

    return trajectory;
  }

  /**
   * Gets the distance of a cargo from the robot in meters based on the size of
   * the cargo in the camera.
   * 
   * @param cargoPxHeight The height in px of the bounding box for the ball seen
   *                      by the camera.
   * @return The distance in meters of the cargo from the robot.
   */
  public static double getCargoDistance(double cargoPxHeight) {

    double table[][] = AIRobotConstants.heightDistanceTable;

    double lowerHeight = 0;
    double lowerDistance = 0;
    double higherHeight = 0.01; // Shouldn't be necessary but just in case
    double higherDistance = 0;

    double firstPxHeight = table[0][0];
    double secondPxHeight = table[1][0];
    double lastPxHeight = table[table.length - 1][0];
    double secondTolastPxHeight = table[table.length - 2][0];

    double firstDistance = table[0][1];
    double secondDistance = table[1][1];
    double lastDistance = table[table.length - 1][1];
    double secondTolastDistance = table[table.length - 2][1];

    double cargoDistance = 0;

    // Handles if the cargo is less than the table's least distance
    if (cargoPxHeight < firstPxHeight) {
      // Uses slope from first 2 points to get distance
      double slope = (secondDistance - firstDistance) / (secondPxHeight - firstPxHeight);
      cargoDistance = slope * (firstPxHeight - cargoPxHeight) + firstDistance;
      // Handles if the cargo is greater than the table's max distance
    } else if (cargoPxHeight > lastPxHeight) {
      // Uses slope from last 2 points to get distance
      double slope = (lastDistance - secondTolastDistance) / (lastPxHeight - secondTolastPxHeight);
      cargoDistance = slope * (cargoPxHeight - lastPxHeight) + lastDistance;
    } else {
      // Gets the closest values below and above the desired value
      for (int i = 0; i < table.length; i++) {
        // Checks if the height is in between 2 points in the table
        if (table[i][0] <= cargoPxHeight && table[i + 1][0] > cargoPxHeight) {
          lowerHeight = table[i][0];
          lowerDistance = table[i][1];
          higherHeight = table[i + 1][0];
          higherDistance = table[i + 1][1];
          break;
        }
      }

      // Gets slope connecting points
      double slope = (higherDistance - lowerDistance) / (higherHeight - lowerHeight);
      cargoDistance = (slope * (cargoPxHeight - lowerHeight) + lowerDistance);

    }

    return cargoDistance;

  }

  /**
   * Gets the x position of the cargo relative to the robot. E.g. 3 meters to the
   * right.
   * 
   * @param cargoPxHeight                   The height in px of the bounding box
   *                                        for the cargo seen by the camera.
   * @param pxFromCameraCenterToCargoCenter The number of pixels from the vertical
   *                                        center of the camera's feed to the
   *                                        vertical center of the bounding box
   *                                        for the cargo seen by the camera.
   * @return The x position of the cargo in meters relative to the robot.
   */
  public static double getCargoXPos(double cargoPxHeight, double pxFromCameraCenterToCargoCenter) {
    // Converts px to meters based on the size of the cargo
    double pxToMeters = AIRobotConstants.cargoDiameterMeters / cargoPxHeight;
    double x = pxFromCameraCenterToCargoCenter * pxToMeters;
    return x;
  }

  /**
   * Gets the y position of the cargo relative to the robot. E.g. 2 meters
   * forward.
   * 
   * @param cargoXPos     The x position of the cargo in meters relative to the
   *                      robot.
   * @param cargoDistance The distance in meters of the cargo from the robot.
   * @return The x position of the cargo in meters relative to the robot.
   */
  public static double getCargoYPos(double cargoXPos, double cargoDistance) {
    // Uses trig to get the y pos
    double theta = Math.acos(cargoXPos / cargoDistance);
    double y = Math.sin(theta) * cargoDistance;
    return y;
  }

  /**
   * Gets the theta of the cargo, or the rotation that the robot has to make to face the cargo head on.
   * 
   * @param cargoXPos     The x position of the cargo in meters relative to the
   *                      robot.
   * @param cargoDistance The distance in meters of the cargo from the robot.
   * @return The theta of the cargo in radians.
   */
  public static double getCargoTheta(double cargoXPos, double cargoDistance) {
    // Uses trig to get theta
    double cargoTheta = Math.asin(cargoXPos / cargoDistance);
    return cargoTheta;
  }

}