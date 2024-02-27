// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.NeoLibrary;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

/** Add your docs here. */
public class SparkMaxConfigurator {

    private final double NOMINAL_VOLTAGE = 12.2;
    private final IdleMode idleMode = IdleMode.kCoast;
    private final Boolean inversionState = false;
    private final double rampRate = 0.05;


    public SparkMaxConfigurator(CANSparkMax sparky) {
        sparky.restoreFactoryDefaults();
        sparky.clearFaults();

        sparky.setIdleMode(idleMode);
        sparky.setInverted(inversionState);
        sparky.setOpenLoopRampRate(rampRate);

        sparky.enableVoltageCompensation(NOMINAL_VOLTAGE);
        sparky.setSmartCurrentLimit(35);

        sparky.burnFlash();
    }

    public SparkMaxConfigurator(CANSparkMax sparky, IdleMode idleMode, Boolean inversionState, Double rampRate) {
        sparky.restoreFactoryDefaults();
        sparky.clearFaults();

        sparky.setIdleMode(idleMode);
        sparky.setInverted(inversionState);
        sparky.setOpenLoopRampRate(rampRate);

        sparky.enableVoltageCompensation(NOMINAL_VOLTAGE);
        sparky.setSmartCurrentLimit(35);

        sparky.burnFlash();
    }

}
