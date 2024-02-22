package frc.robot.vectorfields;

import frc.robot.ultrashot.Point2D;

public class VectorFieldConstants {
    public static final Point2D[] antiNodes = {
        new Point2D(4.0, 3.0),
        new Point2D(-3.0, -2.0),
        new Point2D(1.0, -4.0)
    };

    public static final Point2D node = new Point2D(0, 0);

    public static final double attraction = 1;
    public static final double repulsion = 0.8;


}
