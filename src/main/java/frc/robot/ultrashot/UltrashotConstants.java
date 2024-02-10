package frc.robot.ultrashot;

public class UltrashotConstants {

    public static double shooterVelocity = 14.6958468347; // m/s
    public static double shooterLength = 0.4303; // m
    public static double localGravity = 9.79327; // m/s^2
    public static double airDrag = 0.247; // air resistance coeffiecient

    public static Point3D robot = new Point3D(3, 3, 0);
    public static Point3D robotVelocity = new Point3D(1, 1, 0);
    public static Point3D speaker = new Point3D(0.3556, 5.547868, 2.06986103781);
    public static Point3D shooterAxisInRobotSpace = new Point3D(0.0953, 0, 0.3518);
}
