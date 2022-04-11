// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.autonomous.AutoShoot;
import frc.robot.commands.autonomous.TwoBallAutonomousCommand;
import frc.robot.commands.climb.ClimbWithButtons;
import frc.robot.commands.intake.EjectCommand;
import frc.robot.commands.intake.IntakeWithTower;
import frc.robot.commands.shooter.Eject;
import frc.robot.commands.shooter.EmergencyShoot;
import frc.robot.commands.shooter.RevAndTurnShoot;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.testing.ClimbManualPairedPIDControl;
import frc.robot.commands.testing.ShooterPIDtesting;
import frc.robot.commands.tower.SetTowerMotorSpeed;
import frc.robot.commands.tower.TowerIntake;
import frc.robot.subsystems.*;

import java.util.List;
import java.util.function.DoubleSupplier;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  //  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final LimelightSubsystem m_Limelight = LimelightSubsystem.getInstance();
  private final TowerSubsystem m_tower = new TowerSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final LEDsSubsystem m_LEDs = new LEDsSubsystem();

  private final Command fiveBallAuto = new AutoCommand(m_shooter, m_tower, m_robotDrive, m_LEDs, m_intakeSubsystem);
  private final Command twoBallAuto = new TwoBallAutonomousCommand(m_shooter, m_tower, m_robotDrive, m_LEDs, m_intakeSubsystem);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

//  private final Command twoBallAuto;
//  private final Command noAuto;

//  private final SendableChooser<Command> chooser = new SendableChooser<Command>();

  // The driver's controller
  private final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick m_buttonController = new Joystick(OIConstants.kButtonControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
//    twoBallAuto = new AutonomousCommand(m_shooter, m_tower, m_robotDrive, m_LEDs, m_intakeSubsystem);
//    noAuto = null;

    // Turn off the limelight lights because they are very bright
//    m_Limelight.turnOffLED();
    m_Limelight.turnOnLED();
    m_LEDs.setLEDsDefault();

    autoChooser.setDefaultOption("5 ball auto", fiveBallAuto);
    autoChooser.addOption("2 ball auto", twoBallAuto);
    SmartDashboard.putData(autoChooser);
//    chooser.
    // m_LEDs.setLEDsRaw(-0.39); // will normally be handled by commands, just for testing.

    configureButtonBindings();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(DoubleSupplier supplierValue) {
    double value = supplierValue.getAsDouble();

    // Deadband
    value = deadband(value, OIConstants.kDriverControllerDeadband);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public void setLEDsDefault() {
    m_LEDs.setLEDsDefault();
  }

  public void setLEDsDisabled() {
    m_LEDs.setLEDsOrange();
  }

  public void teleopInitFunc() {
    DoubleSupplier LEFT_STICK_X = () -> m_driverController.getRawAxis(0);
    DoubleSupplier LEFT_STICK_Y = () -> m_driverController.getRawAxis(1);
    DoubleSupplier RIGHT_STICK_X = () -> m_driverController.getRawAxis(2);
    DoubleSupplier RIGHT_STICK_Y = () -> m_driverController.getRawAxis(3);
    JoystickButton A_BUTTON = new JoystickButton(m_driverController, 2);
    JoystickButton RIGHT_TRIGGER = new JoystickButton(m_driverController, 8);

    /*
     * Sets the default command and joystick bindings for the drive train.
     * NOTE: The left stick controls translation of the robot. Turning is controlled by the X axis of the right stick.
     */
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    modifyAxis(LEFT_STICK_Y) * -1 // xAxis
                        * DriveConstants.kMaxSpeedMetersPerSecond,
                    modifyAxis(LEFT_STICK_X) * -1 // yAxis
                        * DriveConstants.kMaxSpeedMetersPerSecond,
                    modifyAxis(RIGHT_STICK_X) * -1 // rot CCW positive
                        * DriveConstants.kMaxRotationalSpeed,
                    !RIGHT_TRIGGER.get()),
            m_robotDrive));
    A_BUTTON.whileHeld(new RevAndTurnShoot(m_shooter, m_Limelight, m_robotDrive, LEFT_STICK_Y, LEFT_STICK_X, m_LEDs));

    new JoystickButton(m_buttonController, 5).whileHeld(
        new Shoot(m_shooter, m_tower, m_Limelight, m_robotDrive, LEFT_STICK_Y, LEFT_STICK_X, m_LEDs));
//    new JoystickButton(m_buttonController, 8).whileHeld(
//        new EmergencyShoot(m_shooter, m_tower, m_Limelight, m_robotDrive, LEFT_STICK_Y, LEFT_STICK_X, m_LEDs));
    new JoystickButton(m_buttonController, 11).whileHeld(new Eject(m_shooter, m_tower));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
//    TODO: figure out what will be mechanically done by competition and get that done in programming
    // Logitech buttons
    JoystickButton X_BUTTON = new JoystickButton(m_driverController, 1);
    JoystickButton B_BUTTON = new JoystickButton(m_driverController, 3);
    JoystickButton Y_BUTTON = new JoystickButton(m_driverController, 4);
    JoystickButton LEFT_BUMPER = new JoystickButton(m_driverController, 5);
    JoystickButton RIGHT_BUMPER = new JoystickButton(m_driverController, 6);
    JoystickButton LEFT_TRIGGER = new JoystickButton(m_driverController, 7);
    POVButton UP_DIRECTION_PAD = new POVButton(m_driverController, 0);
    POVButton RIGHT_DIRECTION_PAD = new POVButton(m_driverController, 90);
    POVButton LEFT_DIRECTION_PAD = new POVButton(m_driverController, 270);
    POVButton DOWN_DIRECTION_PAD = new POVButton(m_driverController, 180);
    JoystickButton LEFT_STICK_DEPRESSED = new JoystickButton(m_driverController, 11);

    RIGHT_DIRECTION_PAD.whenPressed(new InstantCommand(m_robotDrive::zeroHeading));

//    A_BUTTON.whileHeld(new ShooterPIDtesting(m_shooter));
//    LEFT_DIRECTION_PAD.whenPressed(new InstantCommand(m_Limelight::turnOffLED));
//    UP_DIRECTION_PAD.whenPressed(new InstantCommand(m_Limelight::turnOnLED));
//    B_BUTTON.toggleWhenPressed(new ClimbManualPairedPIDControl(m_climbSubsystem, RIGHT_STICK_Y));
//    Y_BUTTON.toggleWhenPressed(new ShooterPIDtesting(m_shooter,m_LEDs,m_tower));


    // Manual Climb
    JoystickButton LClimbUp = new JoystickButton(m_buttonController, 3);
    JoystickButton RClimbUp = new JoystickButton(m_buttonController, 4);
    JoystickButton LClimbDown = new JoystickButton(m_buttonController, 1);
    JoystickButton RClimbDown = new JoystickButton(m_buttonController, 2);
    JoystickButton PneumaticsVertical = new JoystickButton(m_buttonController, 7);
    JoystickButton PneumaticsDown = new JoystickButton(m_buttonController, 6);
    JoystickButton is75Percent = new JoystickButton(m_buttonController, 10);

    new JoystickButton(m_buttonController, 9).toggleWhenPressed(
        new ClimbWithButtons(m_climbSubsystem,
            LClimbUp::get, LClimbDown::get, RClimbUp::get,
            RClimbDown::get, PneumaticsVertical::get, PneumaticsDown::get,
            is75Percent::get, m_LEDs)); // This works

//    new JoystickButton(m_buttonController, 0-9).whileHeld(new SetTowerMotorSpeed(m_tower, -TowerConstants.towerMotorSpeed));

// auto climb stuff
//
//    new JoystickButton(m_buttonController, 0-9).whenPressed(new ClimbFullExtension(m_climbSubsystem));
//    new JoystickButton(m_buttonController, 0-9).whenPressed(new ClimbBottomPosition(m_climbSubsystem))
//    new JoystickButton(m_buttonController, 0-9).whenPressed(new ClimbNextBar(m_climbSubsystem));;

//    B_BUTTON.toggleWhenPressed(new ShooterPIDtesting(m_shooter,m_LEDs,m_tower));

    JoystickButton SUCC_BUTTON = new JoystickButton(m_buttonController, 12);
    // While held for intake
    SUCC_BUTTON.whileHeld( new IntakeWithTower(m_intakeSubsystem, m_tower));
    SUCC_BUTTON.whenReleased(new TowerIntake(m_tower).withTimeout(0.5));

    new JoystickButton(m_buttonController, 8).whileHeld(new SetTowerMotorSpeed(m_tower, m_shooter,
        TowerConstants.towerMotorSpeed));
    // While held for ejecting ball
//    Y_BUTTON.whileHeld(new EjectCommand(m_tower)); // FIXME: Get the button they want
//    Y_BUTTON.whileHeld(new AutoShoot(m_shooter, m_tower, m_Limelight, m_robotDrive, m_LEDs));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
//    return new TwoBallAutonomousCommand(shooter, m_tower, m_robotDrive, m_LEDs, m_intakeSubsystem);

//    return new AutoCommand(m_shooter, m_tower, m_robotDrive, m_LEDs, m_intakeSubsystem);
    return autoChooser.getSelected();

  }
}
