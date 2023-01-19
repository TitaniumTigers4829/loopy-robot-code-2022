// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.autonomous.FiveBallAutoCommand;
import frc.robot.commands.autonomous.FollowRealTimeTrajectory;
import frc.robot.commands.autonomous.RealTimeSwerveControllerCommand;
import frc.robot.commands.autonomous.ThreeBallAutoCommand;
import frc.robot.commands.autonomous.TwoBallAutoCommand;
import frc.robot.commands.autonomous.deprecated.OldThreeBallAutoCommand;
import frc.robot.commands.autonomous.deprecated.OldTwoBallAutoCommand;
import frc.robot.commands.climb.ClimbCommand;
import frc.robot.commands.climb.ClimbHooksToMax;
import frc.robot.commands.climb.ClimbWithButtons;
import frc.robot.commands.drive.Balance;
import frc.robot.commands.drive.FaceForward;
import frc.robot.commands.intake.IntakeWithTower;
import frc.robot.commands.shooter.Eject;
import frc.robot.commands.shooter.EmergencyShoot;
import frc.robot.commands.shooter.RevAndAim;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.tower.SetTowerMotorSpeed;
import frc.robot.commands.tower.TowerIntake;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PiSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import java.util.function.DoubleSupplier;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
// test
public class RobotContainer {

  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  //  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  // private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final LimelightSubsystem m_Limelight = LimelightSubsystem.getInstance();
  private final TowerSubsystem m_tower = new TowerSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final LEDsSubsystem m_LEDs = new LEDsSubsystem();
  private final PiSubsystem m_piSubsystem = new PiSubsystem(m_robotDrive.m_odometry);

  private final Command fiveBallAuto = new FiveBallAutoCommand(m_shooter, m_tower, m_robotDrive,
      m_LEDs,
      m_intakeSubsystem);
  private final Command twoBallAuto = new TwoBallAutoCommand(m_shooter, m_tower, m_robotDrive,
      m_LEDs,
      m_intakeSubsystem);
  private final Command threeBallAuto = new ThreeBallAutoCommand(m_shooter, m_tower, m_robotDrive,
      m_LEDs,
      m_intakeSubsystem);
  private final Command oldTwoBallAuto = new OldTwoBallAutoCommand(m_shooter, m_tower, m_robotDrive,
      m_LEDs, m_intakeSubsystem);
  private final Command oldThreeBallAuto = new OldThreeBallAutoCommand(m_shooter, m_tower,
      m_robotDrive,
      m_LEDs, m_intakeSubsystem);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

//  private final Command twoBallAuto;
//  private final Command noAuto;

//  private final SendableChooser<Command> chooser = new SendableChooser<Command>();

  // The driver's controller
   private final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  // private final XboxController m_driverController = new XboxController(
      // OIConstants.kDriverControllerPort);
  private final Joystick m_buttonController = new Joystick(OIConstants.kButtonControllerPort);
  private final Joystick m_extraButtons = new Joystick(2); // FIXME:make a constant
  // private final Joystick m_buttonController = new Joystick(0);

  // private final Joystick playStationController = new Joystick(0);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_Limelight.turnOnLED();
    m_LEDs.setLEDsDefault();

    autoChooser.setDefaultOption("5 ball auto", fiveBallAuto);
    autoChooser.addOption("2 ball auto", twoBallAuto);
    autoChooser.addOption("3 ball auto", threeBallAuto);
    autoChooser.addOption("Old 2 ball auto", oldTwoBallAuto);
    autoChooser.addOption("Old 3 ball auto", oldThreeBallAuto);
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

  private static double modifyAxisCubed(DoubleSupplier supplierValue) {
    double value = supplierValue.getAsDouble();

    // Deadband
    value = deadband(value, OIConstants.kDriverControllerDeadband);

    // Cube the axis
    value = Math.copySign(value * value * value, value);

    return value;
  }

  private static double modifyAxisQuartic(DoubleSupplier supplierValue) {
    double value = supplierValue.getAsDouble();

    // Deadband
    value = deadband(value, OIConstants.kDriverControllerDeadband);

    // Cube the axis
    value = Math.copySign(value * value * value * value, value);

    return value;
  }

  public void setLEDsDefault() {
    m_LEDs.setLEDsDefault();
  }

  public void setLEDsDisabled() {
//    m_LEDs.setLEDsOrange();
  }

  public void teleopInitFunc() {

       DoubleSupplier LEFT_STICK_X = () -> m_driverController.getRawAxis(0);
  //  DoubleSupplier LEFT_STICK_X = m_driverController::getLeftX;
       DoubleSupplier LEFT_STICK_Y = () -> m_driverController.getRawAxis(1);
  //  DoubleSupplier LEFT_STICK_Y = m_driverController::getLeftY;
       DoubleSupplier RIGHT_STICK_X = () -> m_driverController.getRawAxis(2);
  //  DoubleSupplier RIGHT_STICK_X = m_driverController::getRightX;
       DoubleSupplier RIGHT_STICK_Y = () -> m_driverController.getRawAxis(3);
  //  DoubleSupplier RIGHT_STICK_Y = m_driverController::getRightY;
//        JoystickButton A_BUTTON = new JoystickButton(m_driverController, 2);
   JoystickButton A_BUTTON = new JoystickButton(m_driverController, 1);
//        JoystickButton RIGHT_BUMPER = new JoystickButton(m_driverController, 8);
  // JoystickButton B_BUTTON = new JoystickButton(m_driverController, 2);
   JoystickButton RIGHT_BUMPER = new JoystickButton(m_driverController, 6);
       JoystickButton B_BUTTON = new JoystickButton(m_driverController, 3);
//    JoystickButton B_BUTTON = new JoystickButton(m_driverController, 2);
//    JoystickButton LEFT_BUMPER = new JoystickButton(m_driverController, 5);
    // JoystickButton Y_BUTTON = new JoystickButton(m_driverController, 4);
//
    /*
     * Sets the default command and joystick bindings for the drive train.
     * NOTE: The left stick controls translation of the robot. Turning is controlled by the X axis of the right stick.
     */
   m_robotDrive.setDefaultCommand(
       new RunCommand(
           () ->
               m_robotDrive.drive(
                   modifyAxisCubed(LEFT_STICK_Y) * -1 // xAxis
                       * DriveConstants.kMaxSpeedMetersPerSecond,
                   modifyAxisCubed(LEFT_STICK_X) * -1 // yAxis
                       * DriveConstants.kMaxSpeedMetersPerSecond,
                   modifyAxisCubed(RIGHT_STICK_X) * -1 // rot CCW positive
                       * DriveConstants.kMaxRotationalSpeed,
                   !RIGHT_BUMPER.get()),
           m_robotDrive));
   A_BUTTON.whileHeld(
       new RevAndAim(m_shooter, m_Limelight, m_robotDrive,
           () -> modifyAxisCubed(LEFT_STICK_Y), () -> modifyAxisCubed(LEFT_STICK_X),
           m_LEDs), true);

    B_BUTTON.whileHeld(new Balance(m_robotDrive, 
      () -> modifyAxisCubed(LEFT_STICK_Y), 
      () -> modifyAxisCubed(LEFT_STICK_X), 
      () -> modifyAxisCubed(RIGHT_STICK_X))
    );

    // B_BUTTON.whileHeld(new FollowRealTimeTrajectory(m_robotDrive, () -> !B_BUTTON.get()));

    // Y_BUTTON.whileHeld(new FaceForward(m_robotDrive, () -> modifyAxisCubed(LEFT_STICK_Y) * -1 // xAxis
    //           * DriveConstants.kMaxSpeedMetersPerSecond,
    //       ()->modifyAxisCubed(LEFT_STICK_X) * -1 // yAxis
    //           * DriveConstants.kMaxSpeedMetersPerSecond,
    //       ()->!RIGHT_BUMPER.get())
    // );
//    LEFT_BUMPER.whileHeld(new FaceForward(m_robotDrive, () -> modifyAxisQuartic(LEFT_STICK_Y),
//        () -> modifyAxisQuartic(LEFT_STICK_X), () -> !RIGHT_BUMPER.get()));
//
  //  A_BUTTON.whenReleased(new Shoot(m_shooter, m_tower, m_Limelight, m_robotDrive, ()-> modifyAxisQuartic(LEFT_STICK_Y), ()-> modifyAxisQuartic(LEFT_STICK_X),
  //      m_LEDs, m_intakeSubsystem).withTimeout(0.2));
//
   new JoystickButton(m_buttonController, 5).whileHeld(
       new Shoot(m_shooter, m_tower, m_Limelight, m_robotDrive,
           () -> modifyAxisCubed(LEFT_STICK_Y), () -> modifyAxisCubed(LEFT_STICK_X),
           m_LEDs, m_intakeSubsystem));
    new JoystickButton(m_buttonController, 8).whileHeld(new SetTowerMotorSpeed(m_tower, m_shooter, -1));
  //  new JoystickButton(m_buttonController, 8).whileHeld(
  //      new EmergencyShoot(m_shooter, m_tower, m_Limelight, m_robotDrive, LEFT_STICK_Y, LEFT_STICK_X, m_LEDs));
   new JoystickButton(m_buttonController, 11).whileHeld(new Eject(m_shooter, m_tower));

    // Moves the robot to zeroing position, then zeroes it
    // TODO: stuff
   if (autoChooser.getSelected() == fiveBallAuto) {
    //  m_robotDrive.setGyroOffset(-123);
   }
//      new FaceDegree(m_robotDrive, () -> modifyAxisQuartic(LEFT_STICK_Y),
//          () -> modifyAxisQuartic(LEFT_STICK_X), () -> !RIGHT_BUMPER.get(), 128).withTimeout(1.5)
//          .schedule();
//      new InstantCommand(m_robotDrive::zeroHeading).schedule();
//    } else if (autoChooser.getSelected() == threeBallAuto) {
//      new FaceDegree(m_robotDrive, () -> modifyAxisQuartic(LEFT_STICK_Y),
//          () -> modifyAxisQuartic(LEFT_STICK_X), () -> !RIGHT_BUMPER.get(),
//          145).withTimeout(1.5).schedule();
//      new InstantCommand(m_robotDrive::zeroHeading).schedule();
//    }

    // Resets Odometry On Startup
    m_robotDrive.m_odometry.resetPosition(new Pose2d(), new Rotation2d());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

//      Y
//    X   B
//      A

    // Logitech buttons
//    JoystickButton X_BUTTON = new JoystickButton(m_driverController, 1);
//        JoystickButton X_BUTTON = new JoystickButton(m_driverController, 1);
//    JoystickButton Y_BUTTON = new JoystickButton(m_driverController, 4);
       JoystickButton Y_BUTTON = new JoystickButton(m_driverController, 4);
//        JoystickButton LEFT_BUMPER = new JoystickButton(m_driverController, 5);
//    JoystickButton LEFT_BUMPER = new JoystickButton(m_driverController, 5);
//        JoystickButton RIGHT_BUMPER = new JoystickButton(m_driverController, 6);
//    JoystickButton RIGHT_BUMPER = new JoystickButton(m_driverController, 6);
//        JoystickButton LEFT_TRIGGER = new JoystickButton(m_driverController, 7);
//    JoystickButton LEFT_TRIGGER = new JoystickButton(m_driverController, 7);
//    POVButton UP_DIRECTION_PAD = new POVButton(m_driverController, 0);
   POVButton RIGHT_DIRECTION_PAD = new POVButton(m_driverController, 90);
  //  POVButton LEFT_DIRECTION_PAD = new POVButton(m_driverController, 270);
//    POVButton DOWN_DIRECTION_PAD = new POVButton(m_driverController, 180);

//    JoystickButton LEFT_STICK_DEPRESSED = new JoystickButton(m_driverController, 9);
//        JoystickButton LEFT_STICK_DEPRESSED = new JoystickButton(m_driverController, 11);
//    X_BUTTON.toggleWhenPressed(new setClimbToPos(m_climbSubsystem));
//    B_BUTTON.whenPressed(new InstantCommand(m_climbSubsystem::resetEncoders));
  
  // JoystickButton B_BUTTON = new JoystickButton(m_driverController, 2);
  // JoystickButton X_BUTTON = new JoystickButton(m_driverController, 3);

  RIGHT_DIRECTION_PAD.whenPressed(new InstantCommand(m_robotDrive::zeroHeading));

//    A_BUTTON.whileHeld(new ShooterPIDtesting(m_shooter));
//    LEFT_DIRECTION_PAD.whenPressed(new InstantCommand(m_Limelight::turnOffLED));
//    UP_DIRECTION_PAD.whenPressed(new InstantCommand(m_Limelight::turnOnLED));
//    B_BUTTON.toggleWhenPressed(new ClimbManualPairedPIDControl(m_climbSubsystem, RIGHT_STICK_Y));
//    Y_BUTTON.toggleWhenPressed(new ShooterPIDtesting(m_shooter,m_LEDs,m_tower));

//    double led_value = 0;
//    LEFT_DIRECTION_PAD.toggleWhenPressed(new testLEDs(m_LEDs, led_value));

    // Manual Climb
    JoystickButton LClimbUp = new JoystickButton(m_buttonController, 3);
    JoystickButton RClimbUp = new JoystickButton(m_buttonController, 4);
    JoystickButton LClimbDown = new JoystickButton(m_buttonController, 1);
    JoystickButton RClimbDown = new JoystickButton(m_buttonController, 2);
    JoystickButton PneumaticsVertical = new JoystickButton(m_buttonController, 6);
    JoystickButton PneumaticsDown = new JoystickButton(m_buttonController, 7);
    JoystickButton is75Percent = new JoystickButton(m_buttonController, 10);

    // JoystickButton followPathPlannerPath = new JoystickButton(m_buttonController, 8);

    // followPathPlannerPath.whenPressed(new FollowTrajectoryPathPlanner(m_robotDrive, PathPlannerConstants.firstTestPath, true));

    // new JoystickButton(m_buttonController, 9).toggleWhenPressed(
    //     new ClimbWithButtons(m_climbSubsystem,
    //         LClimbUp::get, LClimbDown::get, RClimbUp::get,
    //         RClimbDown::get, PneumaticsVertical::get, PneumaticsDown::get,
    //         is75Percent::get, m_LEDs)); // This works

    // new JoystickButton(m_buttonController, 11).whenPressed(new InstantCommand(m_climbSubsystem::resetEncoders));

    
    // JoystickButton saveZeroButton = new JoystickButton(m_extraButtons, 3);
    JoystickButton goToZeroButton = new JoystickButton(m_extraButtons, 2);
    JoystickButton autoClimbButton = new JoystickButton(m_extraButtons, 4);

    // goToZeroButton.whileHeld(new ClimbHooksToMax(m_climbSubsystem));
    // autoClimbButton.whileHeld(new ClimbCommand(m_climbSubsystem));

    // new JoystickButton(m_buttonController, 8).whileHeld(new ClimbSetPos(m_climbSubsystem, ClimbConstants.kClimbMinHeight));
    // new JoystickButton(m_buttonController, 10).toggleWhenPressed(new ClimbCommand(m_climbSubsystem));
    // new JoystickButton(m_buttonController, 11).whenPressed(new InstantCommand(m_climbSubsystem::resetEncoders));

    // JoystickButton triangleButton = new JoystickButton(playStationController, 4);

    // JoystickButton RIGHT_BUMPER = new JoystickButton(m_driverController, 6);

      //  DoubleSupplier LEFT_STICK_X = () -> playStationController.getRawAxis(0);
      //  DoubleSupplier LEFT_STICK_Y = () -> playStationController.getRawAxis(1);
      //  DoubleSupplier RIGHT_STICK_X = () -> playStationController.getRawAxis(2);
      //  DoubleSupplier RIGHT_STICK_Y = () -> m_driverController.getRawAxis(3);

      //  m_robotDrive.setDefaultCommand(
      //  new RunCommand(
      //      () ->
      //          m_robotDrive.drive(
      //              modifyAxisCubed(LEFT_STICK_Y) * -1 // xAxis
      //                  * DriveConstants.kMaxSpeedMetersPerSecond,
      //              modifyAxisCubed(LEFT_STICK_X) * -1 // yAxis
      //                  * DriveConstants.kMaxSpeedMetersPerSecond,
      //              modifyAxisCubed(RIGHT_STICK_X) * -1 // rot CCW positive
      //                  * DriveConstants.kMaxRotationalSpeed,
      //              !RIGHT_BUMPER.get()),
      //      m_robotDrive));


//    new JoystickButton(m_buttonController, 0-9).whileHeld(new SetTowerMotorSpeed(m_tower, -TowerConstants.towerMotorSpeed));

// auto climb stuff
//
//    new JoystickButton(m_buttonController, 8).whenPressed(new ClimbFullExtension(m_climbSubsystem));
//    new JoystickButton(m_buttonController, 7).whenPressed(new ClimbBottomPosition(m_climbSubsystem));
//    new JoystickButton(m_buttonController, 6).whenPressed(new ClimbNextBar(m_climbSubsystem));
//    X_BUTTON.whenPressed(new InstantCommand(m_climbSubsystem::resetEncoders));

//    B_BUTTON.toggleWhenPressed(new ShooterPIDtesting(m_shooter,m_LEDs,m_tower));

   JoystickButton SUCC_BUTTON = new JoystickButton(m_buttonController, 12);
//    // While held for intake
   SUCC_BUTTON.whileHeld(new IntakeWithTower(m_intakeSubsystem, m_tower));
   SUCC_BUTTON.whenReleased(new TowerIntake(m_tower).withTimeout(1));

//    new JoystickButton(m_buttonController, 8).whileHeld(new SetTowerMotorSpeed(m_tower, m_shooter,
//        -1));
    // B_BUTTON.whileHeld(new FollowPiTrajectory(m_robotDrive, m_piSubsystem));

    final ProfiledPIDController thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // B_BUTTON.whileHeld(
    //   new ParallelCommandGroup(
    //     new RealTimeSwerveControllerCommand(
    //       m_robotDrive,
    //       m_piSubsystem,
    //       m_robotDrive::getPose, // Functional interface to feed supplier
    //       DriveConstants.kDriveKinematics,

    //       // Position controllers
    //       new PIDController(AutoConstants.kPXController, 0, 0),
    //       new PIDController(AutoConstants.kPYController, 0, 0),
    //       thetaController,
    //       m_robotDrive::setModuleStates),

    //       new IntakeWithTower(m_intakeSubsystem, m_tower)
    //   )
    // );
    // X_BUTTON.whileHeld(new SetTowerMotorSpeed(m_tower, m_shooter, -0.5));


    Y_BUTTON.whenPressed(
       new InstantCommand(() -> m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))));
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
//    return new OldTwoBallAutoCommand(shooter, m_tower, m_robotDrive, m_LEDs, m_intakeSubsystem);
//    return new FiveBallAutoCommand(m_shooter, m_tower, m_robotDrive, m_LEDs, m_intakeSubsystem);
//    return new OldThreeBallAutoCommand(m_shooter, m_tower, m_robotDrive, m_LEDs, m_intakeSubsystem);
//    return new TwoBallAutoCommand(m_shooter, m_tower, m_robotDrive, m_LEDs, m_intakeSubsystem);
    return autoChooser.getSelected();
  }
}
