// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;


public class DriveSubsystem extends SubsystemBase {

    // The gyro sensor
    private final Gyro m_gyro = new AHRS(SPI.Port.kMXP);

    // Don't really know why this is necessary
//  private NetworkTableEntry xSpeedEntry =
//      swerveTab.add("xBox xSpeed", 0)
//          .getEntry();
//
//  private NetworkTableEntry ySpeedEntry =
//      swerveTab.add("xBox ySpeed", 0)
//          .getEntry();
//
//  private NetworkTableEntry rotEntry =
//      swerveTab.add("xBox rot", 0)
//          .getEntry();
//
//  private NetworkTableEntry frontLeftStateEntry =
//      swerveTab.add("FL State v", 0)
//          .getEntry();
//
//  private NetworkTableEntry frontRightStateEntry =
//      swerveTab.add("FR State v", 0)
//          .getEntry();
//
//  private NetworkTableEntry gyroEntry =
//      swerveTab.add("Gyro Heading", 0)
//          .getEntry();
    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry =
            new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());
    private final ShuffleboardTab moduleTab = Shuffleboard.getTab("Module Info");
    private final SwerveModule m_frontLeft =
            new SwerveModule(
                    DriveConstants.kFrontLeftDriveMotorPort,
                    DriveConstants.kFrontLeftTurningMotorPort,
                    DriveConstants.kFrontLeftTurningEncoderPort,
                    DriveConstants.kFrontLeftAngleZero,
                    DriveConstants.kFrontLeftTurningEncoderReversed,
                    DriveConstants.kFrontLeftDriveEncoderReversed,
                    moduleTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                            .withSize(4, 8)
                            .withPosition(0, 0));
    private final SwerveModule m_rearLeft =
            new SwerveModule(
                    DriveConstants.kRearLeftDriveMotorPort,
                    DriveConstants.kRearLeftTurningMotorPort,
                    DriveConstants.kRearLeftTurningEncoderPort,
                    DriveConstants.kRearLeftAngleZero,
                    DriveConstants.kRearLeftTurningEncoderReversed,
                    DriveConstants.kRearLeftDriveEncoderReversed,
                    moduleTab.getLayout("Rear Left Module", BuiltInLayouts.kList)
                            .withSize(4, 8)
                            .withPosition(4, 0));
    private final SwerveModule m_frontRight =
            new SwerveModule(
                    DriveConstants.kFrontRightDriveMotorPort,
                    DriveConstants.kFrontRightTurningMotorPort,
                    DriveConstants.kFrontRightTurningEncoderPort,
                    DriveConstants.kFrontRightAngleZero,
                    DriveConstants.kFrontRightTurningEncoderReversed,
                    DriveConstants.kFrontRightDriveEncoderReversed,
                    moduleTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                            .withSize(4, 8)
                            .withPosition(8, 0));
    private final SwerveModule m_rearRight =
            new SwerveModule(
                    DriveConstants.kRearRightDriveMotorPort,
                    DriveConstants.kRearRightTurningMotorPort,
                    DriveConstants.kRearRightTurningEncoderPort,
                    DriveConstants.kRearRightAngleZero,
                    DriveConstants.kRearRightTurningEncoderReversed,
                    DriveConstants.kRearRightDriveEncoderReversed,
                    moduleTab.getLayout("Rear Right Module", BuiltInLayouts.kList)
                            .withSize(4, 8)
                            .withPosition(12, 0));

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                m_gyro.getRotation2d(),
                m_frontLeft.getState(),
                m_rearLeft.getState(),
                m_frontRight.getState(),
                m_rearRight.getState());

//    SmartDashboard.putString("m_frontLeft", m_frontLeft.getState().toString());
//    SmartDashboard.putString("m_rearLeft", m_rearLeft.getState().toString());
//    SmartDashboard.putString("m_frontRight", m_frontRight.getState().toString());
//    SmartDashboard.putString("m_rearRight", m_rearRight.getState().toString());
        SmartDashboard.putString("odometry", m_odometry.getPoseMeters().toString());
        m_frontLeft.periodic_func();
        m_rearRight.periodic_func();
        m_rearLeft.periodic_func();
        m_frontRight.periodic_func();
    }

    public double heading() {
        return m_gyro.getAngle() % 360;
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SmartDashboard.putBoolean("Field Relative:", fieldRelative);
        SwerveModuleState[] swerveModuleStates =
                DriveConstants.kDriveKinematics.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                                : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
        SmartDashboard.putString("Front Left desired state: ", swerveModuleStates[0].toString());
        SmartDashboard.putString("Front Right desired state: ", swerveModuleStates[1].toString());

    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
//  public double getHeading() {
//    return m_gyro.getRotation2d().getDegrees();
//  }

//  /**
//   * Returns the turn rate of the robot.
//   *
//   * @return The turn rate of the robot, in degrees per second
//   */
//  public double getTurnRate() {
////    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
//    return m_gyro.getRate();
//  }

}
