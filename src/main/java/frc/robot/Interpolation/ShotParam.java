// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Interpolation;

public class ShotParam {
    private double shooterAngle;
    private double flywheelSpeed;

    public ShotParam(double shooterAngle, double flywheelSpeed) {
        this.shooterAngle = shooterAngle;
        this.flywheelSpeed = flywheelSpeed;
    }

    public double getShooterAngle() {
        return shooterAngle;
    }

    public double getFlywheelSpeed() {
        return flywheelSpeed;
    }

    public ShotParam interpolate(double distance, ShotParam end) {
        double endSpeed = end.getFlywheelSpeed();
        double endAngle = end.getShooterAngle();

        return new ShotParam(lerp(distance, getShooterAngle(), endAngle), lerp(distance, getFlywheelSpeed(), endSpeed));
    }

    private double lerp(double x, double a, double b) {
        return a + (x * (b - a));
    }

    public String toString() {
        return String.format("%s %s", getShooterAngle(), getFlywheelSpeed());
    }
}
