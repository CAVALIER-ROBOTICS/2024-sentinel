// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import frc.robot.Constants.LEDConstants;

// // Creates and manages a Rev Blinkin
// public class Blinkin {

//     Spark blinkin;

//     public Blinkin() {
//         blinkin = new Spark(LEDConstants.BLINKIN_ID);
//     }

//     public void setBlink(double color) {
//         blinkin.set(color);
//     }

//     public void boot() {
//         blinkin.set(0.61);
//     }

//     public void isEnabledDisabled(Boolean is) {
//         if (is) {blinkin.set(LEDConstants.enabled);}
//         else {blinkin.set(LEDConstants.disabled);}
//     }

//     public void isError() {
//         if ((DriverStation.getMatchType() == DriverStation.MatchType.Qualification) 
//         || (DriverStation.getMatchType() == DriverStation.MatchType.Elimination)) 
//         {if (!DriverStation.isFMSAttached()) {setBlink(LEDConstants.error);}}
//         else if (!DriverStation.isDSAttached()) {setBlink(LEDConstants.error);}
//     }
// }
