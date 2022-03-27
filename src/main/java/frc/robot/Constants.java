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

    public static final int kFrontLeftTurningMotorPort = 1;
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

    public static final double kTrackWidth = 0.57785;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.57785;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;


    // Values to scale joystick inputs to desired states.
    public static final double kMaxSpeedMetersPerSecond = 4.5; // LOCKED IN
    public static final double kMaxRotationalSpeed = 3 * Math.PI; // TODO: make sure this is right... (maybe should be radians)

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 0.2; // FIXME with sysid
    public static final double kvVoltSecondsPerMeter = 2.75; // FIXME with sysid
    public static final double kaVoltSecondsSquaredPerMeter = 0.15; // FIXME with sysid


    public static final double ksTurning = 0.77; // LOCKED IN!  -----  old 0.66202
    public static final double kvTurning = 0.75; // 3.0052
    public static final double kaTurning = 0; // Default to zero
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2;
  }

  public static final class ModuleConstants {

    public static final double kDriveGearRatio = 7.13;

    public static final double kPModuleTurnController = 8.3; // TUNE: 8.2142
    public static final double kIModuleTurnController = 0; // DO NOT USE
    public static final double kDModuleTurnController = 0; // TUNE

    // Acceleration could be 8pi to make module get anywhere in 0.5 seconds.
    // Will never reach max velocity, so it can be right at the "top" of the triangle.
    // In this case, that would be 2pi.

    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 3 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 10 * Math.PI;

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

    public static final int kIntakeMotorPort = 5;
    public static final int kIntakeDeployedSolenoidPort = 6;
    public static final int kIntakeRetractedSolenoidPort = 4;

    public static final double kIntakeCustomPower = 0.7; // 0 to 1.0
  }

  public static final class TowerConstants {

    public static final int bottomTowerFeedMotorPort = 14;
    public static final int topTowerFeedMotorPort = 16;

    public static final int bottomTowerSensorPort = 1;
    public static final int topTowerSensorPort = 0;
  }

  public static final class ShooterConstants {

    public static final int kLeftShooterMotorPort = 19;
    public static final int kRightShooterMotorPort = 20;
    public static final int kLeftServoPort = 00;
    public static final int kRightServoPort = 01;

    public static int kMaxShooterSpeedMetersPerSecond = 0 - 9;
    // TODO: This reuses constants, probably a good idea to make these constants have a bigger scope
    public static final double kShooterGearRatio = 7.13;
    public static final double kWheelDiameterMeters = 0.1016; // 4 inches
    public static final double kWheelCircumferenceMeters =
    kWheelDiameterMeters * Math.PI; // C = D * pi
    public static final double kShootertoMetersPerSecond =
        (10 * kWheelCircumferenceMeters) / (kShooterGearRatio * 2048);

    // Pre-programmed shoot values
    public static final double towerMotorSpeed = 0.34;

    public static final double fenderShotHeight = 0.3;
    public static final double fenderShotSpeed = 0.57;
    public static final double tarmacShotHeight = 0.45;
    public static final double tarmacShotSpeed = 0.6;
    public static final double lowShotSpeed = 0.6;
    public static final double lowShotHeight = 0.45;

    // Limelight constants
    public static double cameraHeight = 0 - 9;
    public static double cameraAngle = 0 - 9;
    public static double targetHeight = 8.66;
  }


  public static final class ClimbConstants {

    // General constants
    public static final double kClimbMaxHeight = 1.6081375; // in meters // FIXME
    public static final double kClimbMinHeight = 0.815975; // in meters
    public static final double kClimbSlightlyExtendedHeight = 0.9;

    // NOTE:
    // We only need estimates for the bottom position of the climb arms because they are
    // 'zeroed' in the top position. The bottom position values are just estimates because
    // there is the unpredictable nature of how the climb rope winds up.

    public static final double kClimbLeftMinHeightEncoderEstimate = -3764.00390625; // FIXME
    public static final double kClimbRightMinHeightEncoderEstimate = -3756.97; // FIXME

    // in meters, when do we switch to pure voltage control.
    public static final double kClimbMinPosPIDErrorThreshold = 0.10;

    public static final double kClimbVoltageToApplyAfterPID = -3.5; // in volts
    public static final double kClimbVoltageToHoldBottomPosition = -1; // in volts

    // in meters, how much extra should we un-spool, just to be safe we are at max extension.
    public static final double kClimbMaxPosConfirmationExtraHeight = 0.025;


    // ProfiledPID controller constants
    public static final double kPClimbController = 0.3; // FIXME, TUNE
    public static final double kIClimbController = 0; // DO NOT USE
    public static final double kDClimbController = 0;
    public static final double kMaxClimbSpeedMetersPerSecond = 0.1; // FIXME, TUNE
    public static final double kMaxClimbAccelerationMetersPerSecondSquared = 0.05; // FIXME, TUNE

    // Motor constants
    public static final int kLeftClimbMotorPort = 12;
    public static final int kRightClimbMotorPort = 13;
    // Encoder constants
    public static final int kLeftClimbEncoderPort = 11;
    public static final int kRightClimbEncoderPort = 29;
    // Limit Switch constants
    public static final int kLeftClimbLimitSwitchPort = 2;
    public static final int kRightClimbLimitSwitchPort = 3;
    // Solenoid constants
    public static final int kClimbVerticalSolenoidPort = 5;
    public static final int kClimbAngledSolenoidPort = 7;
  }

  public static final class LEDsConstants {

    public static final int kLEDControllerPort = 3;
  }

  public static final class OIConstants {
    public static final int kButtonControllerPort = 1;

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
