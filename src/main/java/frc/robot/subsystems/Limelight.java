package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Limelight is a camera that emits a bright green light to track
 * reflective tape commonly placed around targets for alignment.
 * This class creates all the necessary objects and methods to
 * retrieve data from the limelight in other classes.
 */
public class Limelight
{
  /**
   * Declaring objects that are used for retrieving data from the limelight.
   */
  private static Limelight instance = null;

  private static NetworkTable table;
  private static NetworkTableEntry tx;
  private static NetworkTableEntry ty;
  private static NetworkTableEntry tv;
  private static NetworkTableEntry ta;
  private static NetworkTableEntry camMode;
  private static NetworkTableEntry ledMode;

  /**
   * Enums allow for values to have labels. This is especially useful
   * when a parameter takes a value that has a specific function associated
   * with said value. With labels, it is clearer what a passed in value does.
   * Example: ledMode will blink the green light if the parameter int is "2."
   * Instead of passing in "2," pass in "LEDMode.BLINK." Still passes in the
   * value of "2," just with a label describing what it does.
   */
  private enum LEDMode
  {
    PIPELINE(0),
    OFF(1),
    BLINK(2),
    ON(3);

    private int modeValue;
    private LEDMode(int modeVal)
    {
      this.modeValue = modeVal;
    }
  }

  private enum CamMode
  {
    VISION(0),
    DRIVER(1);

    private int modeValue;
    private CamMode(int modeVal)
    {
      this.modeValue = modeVal;
    }
  }

  /**
   * Method for other classes to use this class' methods.
   * @return the limelight instance object.
   */
  public static Limelight getInstance()
  {
    if (instance == null)
      instance = new Limelight();

    return instance;
  }

  /**
   * Get limelight data from network table.
   */
  private Limelight()
  {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx"); // Horizontal offset from crosshair to target (-29.8 to 29.8 degrees).
    ty = table.getEntry("ty"); // Vertical offset from crosshair to target (-24.85 to 24.85 degrees).
    tv = table.getEntry("tv"); // Whether the limelight has any valid targets (0 or 1).
    ta = table.getEntry("ta"); // Target area (0% of image to 100% of image).
    ledMode = table.getEntry("ledMode"); // limelight's LED state (0-3).
    camMode = table.getEntry("camMode"); // limelight's operation mode (0-1).
  }

  /**
   * Horizontal offset from crosshair to target.
   * @return offset from -29.8 to 29.8 degrees.
   */
  public double getTargetOffsetX()
  {
    return tx.getDouble(0.0);
  }

  /**
   * Vertical offset from crosshair to target.
   * @return offset from -24.85 to 24.85 degrees.
   */
  public double getTargetOffsetY()
  {
    return ty.getDouble(0.0);
  }

  /**
   * Get whether or not a target is detected.
   * @return true if target is found and false if target is not found.
   */
  public boolean isTargetAvailable()
  {
    return tv.getNumber(0).intValue() == 1 ? true : false;
  }

  /**
   * Get area of detected target.
   * @return target area from 0% to 100%.
   */
  public double getTargetArea()
  {
    return ta.getDouble(0.0);
  }

  /**
   * Method to set the green light's status.
   * @param mode either pipeline, off, blink, or on.
   */
  private void setLEDMode(LEDMode mode)
  {
    ledMode.setNumber(mode.modeValue);
  }

  /**
   * Methods for external classes to change green light's status.
   */
  public void turnOnLED()
  {
    this.setLEDMode(LEDMode.ON);
  }
  public void turnOffLED()
  {
    this.setLEDMode(LEDMode.OFF);
  }
  public void blinkLED()
  {
    this.setLEDMode(LEDMode.BLINK);
  }

  /**
   * Method to set camera mode.
   * @param mode either driver or vision mode.
   */
  private void setCamMode(CamMode mode)
  {
    camMode.setNumber(mode.modeValue);
  }

  /**
   * Method to set video feed in driver mode.
   * Turns off green light and switches camera mode to driver.
   */
  public void setModeDriver()
  {
    this.setLEDMode(LEDMode.OFF);
    this.setCamMode(CamMode.DRIVER);
  }

  /**
   * Method to set video feed in vision mode.
   * Turns on green light and switches camera mode to vision.
   */
  public void setModeVision()
  {
    this.setLEDMode(LEDMode.ON);
    this.setCamMode(CamMode.VISION);
  }

  /**
   * Methods to tell whether the limelight is in driver or vision mode.
   * Driver mode will consist of the LEDs being off and the camera being in color.
   * Vision mode will consist of the LEDs being on and the camera being in black and white.
   */
  private boolean isModeDriver()
  {
    return ledMode.getDouble(0.0) == LEDMode.OFF.modeValue && camMode.getDouble(0.0) == CamMode.DRIVER.modeValue;
  }
  private boolean isModeVision()
  {
    return ledMode.getDouble(0.0) == LEDMode.ON.modeValue && camMode.getDouble(0.0) == CamMode.VISION.modeValue;
  }

  /**
   * Method to toggle the type of video feed.
   */
  public void toggleMode()
  {
    if (this.isModeDriver())
    {
      this.setModeVision();
    }
    else if (this.isModeVision())
    {
      this.setModeDriver();
    }
    else
    {
      this.blinkLED();
    }
  }
}