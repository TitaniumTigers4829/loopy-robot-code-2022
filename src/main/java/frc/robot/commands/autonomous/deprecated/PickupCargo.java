// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.autonomous.deprecated;

// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.PiSubsystem;

// public class PickupCargo extends CommandBase {

//   private final DriveSubsystem driveSubsystem;
//   private final PiSubsystem piSubsystem;
  
//   public Trajectory trajectoryToCargo = null;
//   public FollowPiTrajectory followPiTrajectory;

//   /** Creates a new PickupCargo. */
//   public PickupCargo(DriveSubsystem driveSubsystem, PiSubsystem piSubsystem) {
//     this.driveSubsystem = driveSubsystem;
//     this.piSubsystem = piSubsystem;
//     addRequirements(driveSubsystem, piSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     if (piSubsystem.cargoInView()) {
//       double cargoDistance = piSubsystem.getCargoDistance();
//       double x = piSubsystem.getCargoXPos();
//       double y = piSubsystem.getCargoYPos(x, cargoDistance);
//       double theta = piSubsystem.getCargoTheta(x, cargoDistance);
//       trajectoryToCargo = piSubsystem.generateTrajectory(0, 1, 0);
//       FollowPiTrajectory followPiTrajectory = new FollowPiTrajectory(driveSubsystem, trajectoryToCargo, true);
//       followPiTrajectory.schedule();
//     } else {
//       trajectoryToCargo = null;
//     }
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     followPiTrajectory.cancel();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }