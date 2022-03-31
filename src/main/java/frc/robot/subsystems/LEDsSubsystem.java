package frc.robot.subsystems;

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
        m_leds.set(0.69); //Yellow

    }

    public void setLEDsDefault() {
        m_leds.set(0.63); //Orange

    }

    public void setLEDsShooterLiningUp() {
        m_leds.set(0.93); //White

    }

    public void setLEDsClimbMode() {
        m_leds.set(0.61); //Red
    }

}
