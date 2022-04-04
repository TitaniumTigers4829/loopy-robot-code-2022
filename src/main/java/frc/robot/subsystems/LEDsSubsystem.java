package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class LEDsSubsystem extends SubsystemBase {

    private Spark m_leds;

    public LEDsSubsystem() {
        m_leds = new Spark(Constants.ElectronicsConstants.kLEDPort);
//        if () {
//
//            m_leds.set(0.71);
//
//            }
//        else {
//            m_leds.set(0.65); // Orange
//        }
//
//        if ()
    }

    public void setLEDsReadyToShoot() {
        m_leds.set(0.57); //Pink
    }

    public void setLEDsShooterLiningUp() {
        m_leds.set(0.93); //White
    }

    public void setLEDsBlue(){
        m_leds.set(0.87);
    }
    public void setLEDsRed(){
        m_leds.set(0.61);
    }

    public void setLEDsDefault() {
        if (DriverStation.getAlliance() == Alliance.Red) {
            setLEDsRed();
        } else{
            setLEDsBlue();
        }

    }

    public void setLEDsOrange() {
        m_leds.set(0.61); // Orange
    }

    public void setLEDsClimbing() {
        m_leds.set(0.73);
    }
}
