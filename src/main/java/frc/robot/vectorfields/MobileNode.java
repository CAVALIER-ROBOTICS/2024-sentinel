package frc.robot.vectorfields;

import frc.robot.ultrashot.Point2D;

public class MobileNode {

    private Point2D position, velocity;
    private double attractionRadius, attractionBlur;
    private int ticksSinceUpdate;
    private boolean isFaded;

    public MobileNode() {
        this(RiptideConstants.MOBILE_ATTRACTION_RADIUS, RiptideConstants.MOBILE_ATTRACTION_BLUR);
    }

    public MobileNode(double attractionRadius, double attractionBlur) {
        this.position = new Point2D();
        this.velocity = new Point2D();
        this.attractionRadius = attractionRadius;
        this.attractionBlur = attractionBlur;
        this.ticksSinceUpdate = 0;
        this.isFaded = true;
    }

    // getters ------------------------------------------------------------------------------------------------------------------------------------

    public Point2D getPosition() {
        return this.position;
    }

    public Point2D getVelocity() {
        return this.velocity;
    }

    public double getAttractionRadius() {
        return this.attractionRadius;
    }

    public double getAttractionBlur() {
        return this.attractionBlur;
    }

    public int getTicksSinceUpdate() {
        return this.ticksSinceUpdate;
    }

    public boolean getIsFaded() {
        return this.isFaded;
    }

    // setters -------------------------------------------------------------------------------------------------------------------------------------

    public void setPosition(Point2D position) {
        this.position = position;
    }

    public void setVelocity(Point2D velocity) {
        this.velocity = velocity;
    }

    public void setAttractionRadius(double attractionRadius) {
        this.attractionRadius = attractionRadius;
    }

    public void setAttractionBlur(double attractionBlur) {
        this.attractionBlur = attractionBlur;
    }

    public void setTicksSinceUpdate(int ticksSinceUpdate) {
        this.ticksSinceUpdate = ticksSinceUpdate;
    }

    public void setIsFaded(boolean isFaded) {
        this.isFaded = isFaded;
    }

    // other -------------------------------------------------------------------------------------------------------------------------------------

    public Point2D getVector(Point2D robot) {
        if (!isFaded) {
            return this.getAsStaticNode().getVector(robot);
        }
        else {
            return new Point2D();
        }
    }

    public void update(Point2D position, boolean isVisible) {
        ticksSinceUpdate++;
        //System.out.println(isVisible);
        if (isVisible) {
            setVelocity(Point2D.scalar(Point2D.subtract(position, this.position), 50 / ticksSinceUpdate));
            setPosition(position);
            ticksSinceUpdate = 0;
        }
        else {
            setPosition(Point2D.add(this.position, Point2D.scalar(this.velocity, 0.02 * ticksSinceUpdate)));
        }
        if (ticksSinceUpdate >= RiptideConstants.MAXIMUM_TICKS_TO_DISSOLVE_NODE) {
            this.isFaded = true;
        }
    }

    public StaticNode getAsStaticNode() {
        return new StaticNode(position, attractionRadius, attractionBlur);
    }

    // closest point methods -----------------------------------------------------------------------------------------------------------------------

    public Point2D closest(Point2D[] points) {
        return this.position.closest(points);
    }

    public int closestIndex(Point2D[] points) {
        return this.position.closestIndex(points);
    }

    public double closestDistance(Point2D[] points) {
        return this.position.closestDistance(points);
    }
    
}
