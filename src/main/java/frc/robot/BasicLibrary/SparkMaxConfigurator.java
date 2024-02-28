// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BasicLibrary;

import com.revrobotics.CANSparkBase.ExternalFollower;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

/** Add your docs here. */
public class SparkMaxConfigurator {

    private final CANSparkMax sparky;

    private final double NOMINAL_VOLTAGE = 12.2;
    private final IdleMode idleMode = IdleMode.kCoast;
    private final Boolean inversionState = false;
    private final double rampRate = 0.05;

    /**
     * Creates a new SparkMax object
     * 
     * @param sparkMax CAN ID of the SparkMax
     */
    public SparkMaxConfigurator(int sparkMax) {
        sparky = new CANSparkMax(sparkMax, MotorType.kBrushless);
        sparky.restoreFactoryDefaults();
        sparky.clearFaults();

        sparky.setIdleMode(idleMode);
        sparky.setInverted(inversionState);
        sparky.setOpenLoopRampRate(rampRate);

        sparky.enableVoltageCompensation(NOMINAL_VOLTAGE);
        sparky.setSmartCurrentLimit(35);

        sparky.burnFlash();
    }

    /**
     * Creates a new SparkMax object
     * 
     * @param sparkMax CAN ID of the SparkMax
     * @param idleMode IdleMode.kCoast/kBrake
     * @param inversionState Boolean if the motor should be inverted
     * @param rampRate Miniumum time in seconds allowed for the motor to go from 0 to full throttle
     */
    public SparkMaxConfigurator(int sparkMax, IdleMode idleMode, Boolean inversionState, Double rampRate) {
        sparky = new CANSparkMax(sparkMax, MotorType.kBrushless);
        sparky.restoreFactoryDefaults();
        sparky.clearFaults();

        sparky.setIdleMode(idleMode);
        sparky.setInverted(inversionState);
        sparky.setOpenLoopRampRate(rampRate);

        sparky.enableVoltageCompensation(NOMINAL_VOLTAGE);
        sparky.setSmartCurrentLimit(35);

        sparky.burnFlash();
    }

    /**
     * Interface for setting the speed of the motor
     * 
     * @param speed Value between -1.0 & 1.0
     */
    public void set(double speed) {
        sparky.set(speed);
    }

    /**
     * Causes this controller output to mirror another CANSparkMax
     * 
     * @param leader The CAN ID of the motor to mirror
     * @param inversionState If the output should be inverted
     */
    public void follow(int leader, Boolean inversionState) {
        sparky.follow(ExternalFollower.kFollowerSpark, leader, inversionState);
    }

    /**
     * Method to return an object to interface with the intergrated hall sensor in brushless motors
     * 
     * @return Integrated relative encoder object to interface with
     */
    public RelativeEncoder getEncoder() {
        return sparky.getEncoder();
    }

    /**
     * Sets the controllers output voltage, usually <= 12
     * @param voltage the desired output in volts
     */
    public void setVoltage(double voltage) {
        sparky.setVoltage(voltage);
    }

    /**
     * Stops motor movement, Motor can be moved again by calling set method without re enabling
     */
    public void stopMotor() {
        sparky.stopMotor();
    }

    /**
     * Causes current thread to wait for timeMillis, until <em>notified</em> or <em>interrupted</em>
     * 
     * @param timeMillis the maximum time to wait in milliseconds
     */
    public void waitMilliSeconds(long timeMillis) {
        try { sparky.wait(timeMillis); } catch (InterruptedException e) {}
    }

    /**
     * @return Gets the SparkMax's current output in Amps
     */
    public double getOutputCurrent() {
        return sparky.getOutputCurrent();
    }

    /**
     * @return Gets the SparkMax's voltage input in Volts
     */
    public double getVoltage() {
        return sparky.getBusVoltage();
    }

    /**
     * @return The SparkMax's set speed between -1.0 & 1.0
     */
    public double getSpeed() {
        return sparky.get();
    }

}
