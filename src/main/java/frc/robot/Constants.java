// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class ElectronicsConstants {

    public static PneumaticsModuleType kPneumaticsModuleType = PneumaticsModuleType.CTREPCM;
  }


  public static final class DriveConstants {

    public static final int kFrontLeftDriveMotorPort = 24;
    public static final int kRearLeftDriveMotorPort = 6;
    public static final int kFrontRightDriveMotorPort = 4;
    public static final int kRearRightDriveMotorPort = 23;

    public static final int kFrontLeftTurningMotorPort = 21;
    public static final int kRearLeftTurningMotorPort = 7;
    public static final int kFrontRightTurningMotorPort = 3;
    public static final int kRearRightTurningMotorPort = 25;

    public static final int kFrontLeftTurningEncoderPort = 22;
    public static final int kRearLeftTurningEncoderPort = 10;
    public static final int kFrontRightTurningEncoderPort = 9;
    public static final int kRearRightTurningEncoderPort = 8;

    public static final double kFrontLeftAngleZero = 79.45; // FIXME: Add angle offset
    public static final double kRearLeftAngleZero = 121.38; // FIXME: Add angle offset
    public static final double kFrontRightAngleZero = -104.68; // FIXME: Add angle offset
    public static final double kRearRightAngleZero = -118.30; // FIXME: Add angle offset

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kRearRightDriveEncoderReversed = true;

    public static final double kTrackWidth = 0.57785; // FIXME
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.57785; // FIXME
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;


    // Values to scale joystick inputs to desired states.
    public static final double kMaxSpeedMetersPerSecond = 10;
    public static final double kMaxRotationalSpeedMetersPerSecond = 10; // TODO: make sure this is right... (maybe should be radians)

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 0.2; // FIXME with sysid
    public static final double kvVoltSecondsPerMeter = 2.75; // FIXME with sysid
    public static final double kaVoltSecondsSquaredPerMeter = 0.15; // FIXME with sysid


    public static final double ksTurning = 0.77; // LOCKED IN!  -----  old 0.66202
    public static final double kvTurning = 0.6; // 3.0052
    public static final double kaTurning = 0; // Default to zero
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2;
  }

  public static final class ModuleConstants {

    public static final double kDriveGearRatio = 7.13;

    public static final double kPModuleTurnController = 1; // TUNE: 8.2142
    public static final double kIModuleTurnController = 0; // DO NOT USE
    public static final double kDModuleTurnController = 0; // TUNE

    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 4 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 4 * Math.PI;

    public static final double kPModuleDriveController = 0; // TUNE
    public static final double kIModuleDriveController = 0; // DO NOT USE
    public static final double kDModuleDriveController = 0;


    public static final int kDriveFXEncoderCPR = 2048;
    public static final int kTurningCANcoderCPR = 4096;
    public static final double kWheelDiameterMeters = 0.1016; // 4 inches
    public static final double kWheelCircumferenceMeters =
        kWheelDiameterMeters * Math.PI; // C = D * pi
    public static final double kDrivetoMetersPerSecond =
        (10 * kWheelCircumferenceMeters) / (kDriveGearRatio * 2048);
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorPort = 99;
    public static final int kIntakeRetractedSolenoidPort = 4;
    public static final int kIntakeDeployedSolenoidPort = 6;

    public static final double kIntakeCustomPower = 0.7; // 0 to 1.0
  }

  public static final class TowerConstants {
    public static final int bottomTowerFeedMotorPort = 0-9;
    public static final int topTowerFeedMotorPort = 0-9;

    public static final int bottomTowerSensorPort = 0-9;
    public static final int topTowerSensorPort = 0-9;
  }

  public static final class ShooterConstants {
    public static final int kLeftShooterMotorPort = 19;
    public static final int kRightShooterMotorPort = 20;
  }


  public static final class ClimbConstants {
    // General constants
    public static final double kClimbMaxHeight = 0; // in meters // FIXME
    public static final double kClimbMinHeight = 0.86995; // in meters

    // Motor constants
    public static final int kLeftClimbMotorPort = 12;
    public static final int kRightClimbMotorPort = 13;

    // Encoder constants
    public static final int kLeftClimbEncoderPort = 11;
    public static final int kRightClimbEncoderPort = 3;
    public static final double kLeftClimbEncoderOffsetForTopPos = 0; // FIXME ('zero' with arms fully extended)
    public static double kRightClimbEncoderOffsetForTopPos = 0; // FIXME ('zero' with arms fully extended)

    // Limit Switch constants
    public static final int kLeftClimbLimitSwitchPort = 0;
    public static final int kRightClimbLimitSwitchPort = 0;

    // Solenoid constants
    public static final int kClimbVerticalSolenoidPort = 5;
    public static final int kClimbAngledSolenoidPort = 7;

    // ProfiledPID controller constants
    public static final double kPClimbController = 0; // FIXME, TUNE
    public static final double kIClimbController = 0; // DO NOT USE
    public static final double kDClimbController = 0;
    public static final double kMaxClimbSpeedMetersPerSecond = 0.2; // FIXME, TUNE
    public static final double kMaxClimbAccelerationMetersPerSecondSquared = 0.05; // FIXME, TUNE
  }

  public static final class LEDsConstants {

    public static final int kLEDControllerPort = 3;
  }

  public static final class OIConstants {

    public static final int kDriverControllerPort = 0;
    public static final double kDriverControllerDeadband = 0.1;
    public static final int kDriverControllerZeroEncodersButton = 8;
    public static final int kDriverControllerZeroHeadingButton = 9;
  }

  public static final class AutoConstants {

    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
