package frc.robot.ultrashot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UltraShotConstants {

    public static Point3D robot = new Point3D(3, 3, 0);
    public static Point3D axis = new Point3D(0.0953, 0, 0.3518);
    public static Point3D velocity = new Point3D(1, 1, 0);
    
    public static double RED_defaultOffsetX = -12;
    public static double RED_defaultOffsetY = 0;

    public static double BLUE_defaultOffsetX = 0;
    public static double BLUE_defaultOffsetY = 0;

    public static Point3D blueTarget = new Point3D(((9+BLUE_defaultOffsetX)*0.0254), ((218.42-9+BLUE_defaultOffsetY)*0.0254), 2.06986103781); // (9, 218.42) ((1*0.0254), (222.42*0.0254))
    public static Point3D redTarget = new Point3D(((651.25+RED_defaultOffsetX)*0.0254), (5.547868 + Units.inchesToMeters(RED_defaultOffsetY)), 2.06986103781);

    public static AngleStates states = new AngleStates();

    public static double shooterLength = 0.4303; // m
    public static double shooterSpeed = 12.5;
    ; // m/s
    public static double shooterSpeedAuto = 10.7492;
    public static double localGravity = 9.79327; // m/s^2
    public static double airDrag = 1.276; // unitless
    public static double settleTime = 0.1; // s
    public static double futureStepTime = 0.001; // s


    public static String RED_defaultxkey = "X OFFSET RED";
    public static String RED_defaultykey = "Y OFFSET RED";

    public static String BLUE_defaultxkey = "X OFFSET BLUE";
    public static String BLUE_defaultykey = "Y OFFSET BLUE";

    public static void updateTarget() {
        double xOffsetRed = SmartDashboard.getNumber(RED_defaultxkey, RED_defaultOffsetX);
        double yOffsetRed = SmartDashboard.getNumber(RED_defaultykey, RED_defaultOffsetY);

        double xOffsetBlue = SmartDashboard.getNumber(BLUE_defaultxkey, BLUE_defaultOffsetX);
        double yOffsetBlue = SmartDashboard.getNumber(BLUE_defaultykey, BLUE_defaultOffsetY);

        redTarget = new Point3D(((651.25-xOffsetRed)*0.0254), 5.547868 - Units.inchesToMeters(yOffsetRed), 2.06986103781);    
        blueTarget = new Point3D(((9-xOffsetBlue)*0.0254), ((218.42-yOffsetBlue)*0.0254), 2.06986103781); // (9, 218.42) ((1*0.0254), (222.42*0.0254))

    }


}
