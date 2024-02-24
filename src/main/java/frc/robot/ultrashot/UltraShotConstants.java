package frc.robot.ultrashot;

public class UltraShotConstants {

    public static Point3D robot = new Point3D(3, 3, 0);
    public static Point3D axis = new Point3D(0.0953, 0, 0.3518);
    public static Point3D velocity = new Point3D(1, 1, 0);
    public static Point3D blueTarget = new Point3D((9*0.0254), 5.547868, 2.06986103781);
    public static Point3D redTarget = new Point3D(((651.25-9)*0.0254), 5.547868, 2.06986103781);

    public static AngleStates states = new AngleStates();

    public static double shooterLength = 0.4303; // m
    public static double shooterSpeed = 12.01; // m/s
    public static double localGravity = 9.79327; // m/s^2
    public static double airDrag = 1.276; // unitless
    public static double settleTime = 0.1; // s
    public static double omegaStepTime = 0.001; // s
}
