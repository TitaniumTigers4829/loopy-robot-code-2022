package frc.robot.commands.testing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDsSubsystem;

public class testLEDs extends CommandBase {

  private final LEDsSubsystem leds;
  private double value;

  public testLEDs(LEDsSubsystem leds, double value) {
    this.leds = leds;
//    this.value = value;
    addRequirements(leds);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("led value", 0);
  }

  @Override
  public void execute() {
    value = SmartDashboard.getNumber("led value", 0);
    leds.setLEDsManual(value);
  }
}