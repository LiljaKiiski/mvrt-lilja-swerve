package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;

public class TalonFactory {
    
    /**
     * Create and config a new TalonFX
     * 
     * @param id        the id of the Falcon
     * @param inversion the inversion of the Falcon (false to spin forward, true to spin backwards)
     * @return          new TalonFX motor object
     */
    public static TalonFX createTalonFX(int id, boolean inversion) {
        return createTalonFX(id, inversion, null);
    }

     /**
     * Creates a basic TalonFX with basic configurations
     * 
     * @param id        the id of the Falcon on the robot (get from PheonixTuner)
     * @param inversion the inversion of the TalonFX (false to spin forward, true to spin backwards)
     * @param buse_name the name of CAN bus TalonFX is connected to
     * @return          the generated TalonFX object
     */
    public static TalonFX createTalonFX(int id, boolean inversion, String bus_name) {
        TalonFX talon;

        //Create talon (with or without bus name)
        if (bus_name == null){
            talon = new TalonFX(id);

        } else {
            talon = new TalonFX(id, bus_name);
        }

        //Config talon
        talon.configFactoryDefault();
        talon.configSupplyCurrentLimit(Constants.Talon.kCurrentLimit, Constants.Talon.kTimeoutMs);
        talon.configOpenloopRamp(0.4, Constants.Talon.kTimeoutMs);
        talon.setInverted(inversion);
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.Talon.kPIDIdx, Constants.Talon.kTimeoutMs);
        talon.configVoltageCompSaturation(Constants.Talon.kVoltage, Constants.Talon.kTimeoutMs);
        talon.enableVoltageCompensation(true);       

        return talon;
    }
}