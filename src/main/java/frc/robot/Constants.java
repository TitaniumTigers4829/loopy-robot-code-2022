// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 13;
    public static final int kRearLeftDriveMotorPort = 6;
    public static final int kFrontRightDriveMotorPort = 4;
    public static final int kRearRightDriveMotorPort = 23;

    public static final int kFrontLeftTurningMotorPort = 21;
    public static final int kRearLeftTurningMotorPort = 7;
    public static final int kFrontRightTurningMotorPort = 3;
    public static final int kRearRightTurningMotorPort = 12;

    public static final int kFrontLeftTurningEncoderPort = 22;
    public static final int kRearLeftTurningEncoderPort = 10;
    public static final int kFrontRightTurningEncoderPort = 9;
    public static final int kRearRightTurningEncoderPort = 8;

    public static final double kFrontLeftAngleZero = 0; // FIXME: Add angle offset
    public static final double kRearLeftAngleZero = 0; // FIXME: Add angle offset
    public static final double kFrontRightAngleZero = 0; // FIXME: Add angle offset
    public static final double kRearRightAngleZero = 0; // FIXME: Add angle offset

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = true;

    public static final double kTrackWidth = 0.5; // FIXME
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.7; // FIXME
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1; // FIXME with sysid
    public static final double kvVoltSecondsPerMeter = 0.8; // FIXME with sysid
    public static final double kaVoltSecondsSquaredPerMeter = 0.15; // FIXME with sysid
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxRotationalSpeedMetersPerSecond = 3;


    public static final double ksTurning = 0.742; // FIXME feedforward turning
    public static final double kvTurning = 0.216;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2;
  }

  public static final class ModuleConstants {
    // Drive motor -> FX Encoder (2048 units)
    // Turning motor -> CTRE CANcoder (4096 units)
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI; // try 10 if going well
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI; // try 10 if going well

    public static final double kDriveGearRatio = 7.13; // FIXME, I think 7.13 is right tho...
    public static final double kTurningGearRatio = 12.8; // FIXME, might be unnecessary and/or 1.

    public static final double kPModuleTurnController = 1; // TUNE
    public static double kIModuleTurnController = 0;
    public static final double kDModuleTurnController = 0; // TUNE

    public static final double kPModuleDriveController = 1; // TUNE
    public static double kIModuleDriveController = 0;
    public static final double kDModuleDriveController = 0;


    public static final int kDriveFXEncoderCPR = 2048;
    public static final int kTurningCANcoderCPR = 4096;
    public static final double kWheelDiameterMeters = 0.1016; // 4 inches
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI; // C = D * pi
    public static final double kDrivetoMetersPerSecond = (10 * kWheelCircumferenceMeters)/(kDriveGearRatio * 2048);
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriverControllerDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
