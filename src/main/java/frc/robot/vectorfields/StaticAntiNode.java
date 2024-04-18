package frc.robot.vectorfields;

import frc.robot.ultrashot.Point2D;

public class StaticAntiNode {

    private Point2D position;
    private double repulsionRadius, repulsionBlur;

    public StaticAntiNode() {
        this(new Point2D(), RiptideConstants.STATIC_REPULSION_RADIUS, RiptideConstants.STATIC_REPULSION_BLUR);
    }

    public StaticAntiNode(Point2D position) {
        this(position, RiptideConstants.STATIC_REPULSION_RADIUS, RiptideConstants.STATIC_REPULSION_BLUR);
    }

    public StaticAntiNode(Point2D position, double repulsionRadius, double repulsionBlur) {
        this.position = position;
        this.repulsionRadius = repulsionRadius;
        this.repulsionBlur = repulsionBlur;
    }

    // getters ------------------------------------------------------------------------------------------------------------------------------

    public Point2D getPosition() {
        return this.position;
    }

    public double getRepulsionRadius() {
        return this.repulsionRadius;
    }

    public double getRepulsionBlur() {
        return this.repulsionBlur;
    }

    // setters -----------------------------------------------------------------------------------------------------------------------------

    public void setPosition(Point2D position) {
        this.position = position;
    }

    public void setRepulsionRadius(double repulsionRadius) {
        this.repulsionRadius = repulsionRadius;
    }

    public void setRepulsionBlur(double repulsionBlur) {
        this.repulsionBlur = repulsionBlur;
    }

    // other --------------------------------------------------------------------------------------------------------------------------------

    public Point2D getVector(Point2D robot) {
        return Point2D.scalar(Point2D.difference(robot, position), 1.0 / (Math.pow(Point2D.difference(robot, position).getHypot(), 1.0 / repulsionBlur) / repulsionRadius));
    }

    public void update(Point2D position) {
        setPosition(position);
    }

    // closest point methods ----------------------------------------------------------------------------------------------------------------

    public Point2D closest(Point2D[] points) {
        return this.position.closest(points);
    }

    public double closestDistance(Point2D[] points) {
        return this.position.closestDistance(points);
    }
    
}
