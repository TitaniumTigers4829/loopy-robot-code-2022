package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDsConstants;

public class LEDsSubsystem extends  SubsystemBase{
    private final Spark ledController;

    // TODO: create a ENUM of the different LED patterns we want to use.

    public LEDsSubsystem() {
      // Initialize Controller
      ledController = new Spark(LEDsConstants.kLEDControllerPort);
    }

    // TODO: Create a ENUM set method that is easier to use.

    public void setLEDsRaw(double LED) {
      if (LED < 1 && LED > -1) {
        ledController.set(LED);
      } else {
        throw new IllegalArgumentException("The value you tried to set the LEDsSubsystem too is invalid");
    }
  }
}
