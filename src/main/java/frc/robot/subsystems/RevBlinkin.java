// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RevBlinkin extends SubsystemBase {

  Spark blinkin;

  /** Creates a new RevBlinkin. */
  public RevBlinkin() {
    blinkin = new Spark(Constants.blinkinID);
  }

  public void setColor(double color) {
    blinkin.set(color);
  }

  public void setDisabled() {
    blinkin.set(0.59);
  }

  public void setEnabled() {
    blinkin.set(0.75);
  }

  public void isIntake() {
    blinkin.set(-0.41);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
