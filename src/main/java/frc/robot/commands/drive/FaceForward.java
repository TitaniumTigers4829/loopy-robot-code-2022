package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class FaceForward extends CommandBase {
    private final ProfiledPIDController turnProfiledPIDController = new ProfiledPIDController(
            0.1,
            ShooterConstants.turnkI,
            ShooterConstants.turnkD,
            new TrapezoidProfile.Constraints(
                    ShooterConstants.kMaxTurnAngularSpeedRadiansPerSecond,
                    ShooterConstants.kMaxTurnAngularAccelerationRadiansPerSecondSquared)
    );
    
  private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(
    ShooterConstants.ksClimbLineup, ShooterConstants.kvClimbLineup
  );

    private final DriveSubsystem drive;
    private final DoubleSupplier leftStickY;
    private final DoubleSupplier leftStickX;
    private final BooleanSupplier isFieldRelative;

    public FaceForward(DriveSubsystem drive, DoubleSupplier leftStickY, DoubleSupplier leftStickX, BooleanSupplier isFieldRelative) {
        this.drive = drive;
        this.leftStickY = leftStickY;
        this.leftStickX = leftStickX;
        this.isFieldRelative = isFieldRelative;
        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // double headingOffset = drive.heading() % 360;
        // headingOffset = (Math.abs(headingOffset) < 1 ? 0 : headingOffset);
        // double turnOutput = -1 * (turnProfiledPIDController.calculate(headingOffset, 0) + turnFeedforward.calculate(turnProfiledPIDController.getSetpoint().velocity));
        // drive.drive(leftStickY.getAsDouble(), leftStickX.getAsDouble(), turnOutput, isFieldRelative.getAsBoolean());
        // SmartDashboard.putNumber("Output", turnOutput);
        
        double headingError = drive.heading() % 360;

        // If heading error isn't off by much, it won't move
        if (Math.abs(headingError) < 1) headingError = 0;
    
        double turnRobotOutput =
            -1 * 
            turnProfiledPIDController.calculate(headingError, 0)
                + turnFeedforward.calculate(turnProfiledPIDController.getSetpoint().velocity);
    
        drive.drive(leftStickX.getAsDouble(), leftStickY.getAsDouble(), turnRobotOutput, false);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
