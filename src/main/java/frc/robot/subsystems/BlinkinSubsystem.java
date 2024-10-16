// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Damn this thing has a stupid name

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class BlinkinSubsystem extends SubsystemBase {
  Spark blink;
  /** Creates a new BlinkinSubsystem. */
  public BlinkinSubsystem() {
    blink = new Spark(LEDConstants.BLINKIN_ID);
  }

  public void setColor(double color) {
    blink.set(color);
  }

  public void setEnabled(Boolean is) {
      if (is) {blink.set(LEDConstants.enabled);}
      else {blink.set(LEDConstants.disabled);}
  }

  public void isError() {
      if ((DriverStation.getMatchType() == DriverStation.MatchType.Qualification) 
      || (DriverStation.getMatchType() == DriverStation.MatchType.Elimination)) 
      {if (!DriverStation.isFMSAttached()) {setColor(LEDConstants.error);}}
      else if (!DriverStation.isDSAttached()) {setColor(LEDConstants.error);}
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
