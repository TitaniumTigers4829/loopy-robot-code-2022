package frc.robot.commands.testing;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class ShooterPIDtesting extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final LEDsSubsystem leds;
  private final TowerSubsystem tower;
  private double top_target;
  private double bottom_target;
  private double distance;
  private double overshoot_elimination_counter = 0;

  public ShooterPIDtesting (ShooterSubsystem shooterSubsystem, LEDsSubsystem leds, TowerSubsystem tower) {
    this.shooterSubsystem = shooterSubsystem;
    this.leds = leds;
    this.tower = tower;
  }

  public void initialize() {
    SmartDashboard.putNumber("Set bottomRPM", 0);
    SmartDashboard.putNumber("Set topRPM", 0);
    SmartDashboard.putNumber("Set Distance:", 0);

  }

  public void execute() {
    top_target = SmartDashboard.getNumber("Set topRPM", 0);
    bottom_target = SmartDashboard.getNumber("Set bottomRPM", 0);
    distance = Units.feetToMeters(SmartDashboard.getNumber("Set Distance:", 0));

    SmartDashboard.putNumber("Top Shooter Error: ", shooterSubsystem.getTopRPM()-top_target);
    SmartDashboard.putNumber("Bottom Shooter Error: ", shooterSubsystem.getBottomRPM()-bottom_target);

    top_target = calculateRPM(ShooterConstants.topMotorValues, distance);
    bottom_target = calculateRPM(ShooterConstants.bottomMotorValues, distance);

    if (!(top_target == 0 && bottom_target == 0)) {
      shooterSubsystem.setShooterRPMNotImproved(bottom_target, top_target);
    } else {
      shooterSubsystem.setShooterToNeutral();
    }

    if (isReadyToShoot()) {
      leds.setLEDsReadyToShoot();
      tower.setTowerMotorsSpeed(TowerConstants.towerMotorSpeed);
    } else {
      leds.setLEDsShooterLiningUp();
      tower.setTowerMotorsSpeed(0);
    }
  }

  private boolean isReadyToShoot() {

//    if (shooterSubsystem.isShooterWithinAcceptableError(top_target, bottom_target)) {
//      overshoot_elimination_counter = overshoot_elimination_counter + 1;
//    } else {
//      overshoot_elimination_counter = 0;
//    }
//    SmartDashboard.putNumber("overshoot_counter: ", overshoot_elimination_counter);
    return shooterSubsystem.isShooterWithinAcceptableError(top_target, bottom_target);
  }

  private double calculateRPM(double[][] table, double distance) {
    double lowerDistance = 0;
    double lowerSpeed = 0;
    double higherDistance = 0.1; // Shouldn't be necessary but just in case
    double higherSpeed = 0;

    // Gets the closest values below and above the desired value
    for (int i = 0; i < table.length; i++) {
      try {
        if (table[i][0] <= distance && table[i + 1][0] > distance) {
          lowerDistance = table[i][0];
          lowerSpeed = table[i][1];
          higherDistance = table[i + 1][0];
          higherSpeed = table[i + 1][1];
          break;
        }
      } catch (Exception e) {
//        Shouldn't be called, but...
//        Error handling is a thing
        assert table[0] != null;
        lowerDistance = table[0][0];
        lowerSpeed = table[0][1];
        assert table[1] != null;
        higherDistance = table[1][0];
        higherSpeed = table[0][1];
        DriverStation.reportWarning("Error in LimelightSubsystem.calculateRPM(), error: " + e,
            true);
      }
    }

    // Shouldn't be necessary but just in case
    if (table[table.length - 1][0] < distance)
      return table[table.length - 1][1];

    // Gets slope or line connecting points
    double linearSlope = (higherSpeed - lowerSpeed) / (higherDistance - lowerDistance);

    // Uses point slope form to get the xValue
    return (linearSlope * (distance - lowerDistance) + lowerSpeed);
  }
}
