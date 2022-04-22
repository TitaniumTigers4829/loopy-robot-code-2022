package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class FaceDegree extends CommandBase {
    private final ProfiledPIDController turnProfiledPIDController = new ProfiledPIDController(
            ShooterConstants.turnkP,
            ShooterConstants.turnkI,
            ShooterConstants.turnkD,
            new TrapezoidProfile.Constraints(
                    ShooterConstants.kMaxTurnAngularSpeedRadiansPerSecond,
                    ShooterConstants.kMaxTurnAngularAccelerationRadiansPerSecondSquared)
    );

    private final DriveSubsystem drive;
    private final DoubleSupplier leftStickY;
    private final DoubleSupplier leftStickX;
    private final BooleanSupplier isFieldRelative;
    private final double degree;

    public FaceDegree(DriveSubsystem drive, DoubleSupplier leftStickY, DoubleSupplier leftStickX, BooleanSupplier isFieldRelative, double degree) {
        this.drive = drive;
        this.leftStickY = leftStickY;
        this.leftStickX = leftStickX;
        this.isFieldRelative = isFieldRelative;
        this.degree = degree;
        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double headingOffset = drive.heading();
//        headingOffset = (Math.abs(headingOffset) < 1 ? 0 : headingOffset);
        double turnOutput = turnProfiledPIDController.calculate(headingOffset, degree);
        drive.drive(leftStickY.getAsDouble(), leftStickX.getAsDouble(), turnOutput, isFieldRelative.getAsBoolean());
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
