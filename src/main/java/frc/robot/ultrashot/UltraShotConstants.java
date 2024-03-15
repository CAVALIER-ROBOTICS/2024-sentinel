package frc.robot.ultrashot;

import edu.wpi.first.math.util.Units;

public class UltraShotConstants {

    public static final Point3D POINT_3D_SHOOTER_AXIS = new Point3D(0.0953, 0, 0.3518);

    public static final double SHOOTER_LENGTH = 0.4303; // m
    public static final double LOCAL_GRAVITY = 9.79327; // m/s^2
    public static final double AIR_DRAG = 0.247; // unitless
    public static final double FREEDOM_TIME = 0.1; // s
    public static final double MAX_ROBOT_SPEED = 4.2; // m/s

    public static final int NUM_FRAMES_ROBOT_HISTORY = 7;

    public static final Point3D POINT_3D_SPEAKER_BLUE = Point3D.scalar(new Point3D(9, 218.42, 81.5), 0.0254);
    public static final Point3D POINT_3D_SPEAKER_RED = Point3D.scalar(new Point3D(651.25-9, 218.42, 81.5), 0.0254);

    public static final Point3D POINT_3D_AMP_BLUE = new Point3D();
    public static final Point3D POINT_3D_AMP_RED = new Point3D();

    public static final Point3D POINT_3D_CENTER_TRAP_BLUE = new Point3D();
    public static final Point3D POINT_3D_AMP_TRAP_BLUE = new Point3D();
    public static final Point3D POINT_3D_SOURCE_TRAP_BLUE = new Point3D();
    
    public static final Point3D POINT_3D_CENTER_TRAP_RED = new Point3D();
    public static final Point3D POINT_3D_AMP_TRAP_RED = new Point3D();
    public static final Point3D POINT_3D_SOURCE_TRAP_RED = new Point3D();

}
