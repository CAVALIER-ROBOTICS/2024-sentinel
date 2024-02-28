// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.NeoLibrary;

import com.revrobotics.CANSparkBase.ExternalFollower;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

/** Add your docs here. */
public class SparkMaxConfigurator {

    private final CANSparkMax sparky;

    private final double NOMINAL_VOLTAGE = 12.2;
    private final IdleMode idleMode = IdleMode.kCoast;
    private final Boolean inversionState = false;
    private final double rampRate = 0.05;


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

    public void set(double speed) {
        sparky.set(speed);
    }

    public void follow(int leader, Boolean inversionState) {
        sparky.follow(ExternalFollower.kFollowerSpark, leader, inversionState);
    }

}
