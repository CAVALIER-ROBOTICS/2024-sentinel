package frc.robot.vectorfields;

import frc.robot.ultrashot.Point2D;

public class StaticNode {

    private Point2D position;
    private double attractionRadius, attractionBlur;

    public StaticNode() {
        this(new Point2D(), RiptideConstants.STATIC_ATTRACTION_RADIUS, RiptideConstants.STATIC_ATTRACTION_BLUR);
    }

    public StaticNode(Point2D position) {
        this(position, RiptideConstants.STATIC_ATTRACTION_RADIUS, RiptideConstants.STATIC_ATTRACTION_BLUR);
    }

    public StaticNode(Point2D position, double attractionRadius, double attractionBlur) {
        this.position = position;
        this.attractionRadius = attractionRadius;
        this.attractionBlur = attractionBlur;
    }

    // getters

    public Point2D getPosition() {
        return this.position;
    }

    public double getAttractionRadius() {
        return this.attractionRadius;
    }

    public double getAttractionBlur() {
        return this.attractionBlur;
    }

    // setters

    public void setPosition(Point2D position) {
        this.position = position;
    }

    public void setAttractionRadius(double attractionRadius) {
        this.attractionRadius = attractionRadius;
    }

    public void setAttractionBlur(double attractionBlur) {
        this.attractionBlur = attractionBlur;
    }

    // other

    public Point2D getVector(Point2D robot) {
        return Point2D.scalar(Point2D.difference(robot, position), 1.0 / (Math.pow(Point2D.difference(robot, position).getHypot(), 1.0 / attractionBlur) / attractionRadius));
    }

    public void update(Point2D position) {
        setPosition(position);
    }

    public Point2D closest(Point2D[] points) {
        return this.position.closest(points);
    }

    public double closestDistance(Point2D[] points) {
        return this.position.closestDistance(points);
    }
    
}
