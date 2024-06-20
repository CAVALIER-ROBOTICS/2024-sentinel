package frc.robot.vectorfields;

import frc.robot.ultrashot.Point2D;

public class RiptideConstants {

    public static final double STANDARD_SPEED = 1.0;

    public static final int MAXIMUM_NUM_MOBILE_ANTINODES = 5;
    public static final int MAXIMUM_NUM_MOBILE_NODES = 15;

    public static final double STEP_TOLERANCE_ANTINODES = 0.5;
    public static final double STEP_TOLERANCE_NODES = 0.5;

    public static final double STEP_TIME_PROJECTION_ANTINODES = 0.25;

    public static final double STATIC_ATTRACTION_RADIUS = 1.0;
    public static final double STATIC_ATTRACTION_BLUR = 1.0;

    public static final double MOBILE_ATTRACTION_RADIUS = 1.0;
    public static final double MOBILE_ATTRACTION_BLUR = 1.0;

    public static final double STATIC_REPULSION_RADIUS = 1.0;
    public static final double STATIC_REPULSION_BLUR = 0.5;

    public static final double MOBILE_REPULSION_RADIUS = 1.0;
    public static final double MOBILE_REPULSION_BLUR = 1.0;

    public static final int MAXIMUM_TICKS_TO_DISSOLVE_NODE = 120;
    public static final int MAXIMUM_TICKS_TO_DISSOLVE_ANTINODE = 30;

    public static final StaticNode SPEAKER_NODE_BLUE = new StaticNode(new Point2D(0, 0));
    public static final StaticNode SPEAKER_NODE_RED = new StaticNode(new Point2D(0, 0));

    public static final StaticNode AMP_NODE_BLUE = new StaticNode(new Point2D(0, 0));
    public static final StaticNode AMP_NODE_RED = new StaticNode(new Point2D(0, 0));

    public static final StaticNode[] STARTING_NOTES = {
        new StaticNode(new Point2D(0, 0)),
        new StaticNode(new Point2D(0, 0)),
        new StaticNode(new Point2D(0, 0)),
        new StaticNode(new Point2D(0, 0)),
        new StaticNode(new Point2D(0, 0)),
        new StaticNode(new Point2D(0, 0)),
        new StaticNode(new Point2D(0, 0)),
        new StaticNode(new Point2D(0, 0)),
        new StaticNode(new Point2D(0, 0)),
        new StaticNode(new Point2D(0, 0)),
        new StaticNode(new Point2D(0, 0))
    };

    public static final StaticAntiNode[] FIELD_ELEMENTS = {
        new StaticAntiNode(new Point2D(+4.889, +0.000)),
        new StaticAntiNode(new Point2D(+2.660, +1.286)),
        new StaticAntiNode(new Point2D(+2.660, -1.286)),
        new StaticAntiNode(new Point2D(-4.889, +0.000)),
        new StaticAntiNode(new Point2D(-2.660, +1.286)),
        new StaticAntiNode(new Point2D(-2.660, -1.286))
        // new StaticAntiNode(new Point2D(0, 0)),
        // new StaticAntiNode(new Point2D(0, 0)),
        // new StaticAntiNode(new Point2D(0, 0)),
        // new StaticAntiNode(new Point2D(0, 0))
    };
    
}
