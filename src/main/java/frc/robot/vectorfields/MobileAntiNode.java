package frc.robot.vectorfields;

import frc.robot.ultrashot.Point2D;

public class MobileAntiNode {

    private Point2D position, velocity;
    private double repulsionRadius, repulsionBlur;
    private int ticksSinceUpdate;
    private boolean isFaded;

    public MobileAntiNode() {
        this(RiptideConstants.MOBILE_REPULSION_RADIUS, RiptideConstants.MOBILE_REPULSION_BLUR);
    }

    public MobileAntiNode(double repulsionRadius, double repulsionBlur) {
        this.position = new Point2D();
        this.velocity = new Point2D();
        this.repulsionRadius = repulsionRadius;
        this.repulsionBlur = repulsionBlur;
        this.ticksSinceUpdate = 0;
        this.isFaded = true;
    }

    // getters --------------------------------------------------------------------------------------------------------------------------------

    public Point2D getPosition() {
        return this.position;
    }

    public Point2D getVelocity() {
        return this.velocity;
    }

    public double getRepulsionRadius() {
        return this.repulsionRadius;
    }

    public int getTicksSinceUpdate() {
        return this.ticksSinceUpdate;
    }

    public boolean getIsFaded() {
        return this.isFaded;
    }

    // setters ---------------------------------------------------------------------------------------------------------------------------------

    public void setPosition(Point2D position) {
        this.position = position;
    }

    public void setVelocity(Point2D velocity) {
        this.velocity = velocity;
    }

    public void setRepulsionRadius(double repulsionRadius) {
        this.repulsionRadius = repulsionRadius;
    }

    public void setTicksSinceUpdate(int ticksSinceUpdate) {
        this.ticksSinceUpdate = ticksSinceUpdate;
    }

    public void setIsFaded(boolean isFaded) {
        this.isFaded = isFaded;
    }

    // other -----------------------------------------------------------------------------------------------------------------------------------

    public Point2D getVector(Point2D robot) {
        System.out.println("wasa -");
        if (!isFaded) {
            System.out.println("doosa");
            return Point2D.add(this.getAsStaticAntiNode().getVector(robot), this.getProjectionAsStaticAntiNode().getVector(robot));
        }
        else {
            return new Point2D();
        }
    }

    public void update(Point2D position, boolean isVisible) {
        ticksSinceUpdate++;
        if (isVisible) {
            setIsFaded(false);
            setVelocity(Point2D.scalar(Point2D.subtract(position, this.position), 50 / ticksSinceUpdate));
            setPosition(position);
            ticksSinceUpdate = 0;
        }
        else {
            setPosition(Point2D.add(this.position, Point2D.scalar(this.velocity, 0.02 * ticksSinceUpdate)));
        }
        if (ticksSinceUpdate >= RiptideConstants.MAXIMUM_TICKS_TO_DISSOLVE_ANTINODE) {
            this.isFaded = true;
        }
    }

    public StaticAntiNode getAsStaticAntiNode() {
        return new StaticAntiNode(this.position, this.repulsionRadius, this.repulsionBlur);
    }

    public StaticAntiNode getProjectionAsStaticAntiNode() {
        return new StaticAntiNode(Point2D.add(this.position, Point2D.scalar(this.velocity, RiptideConstants.STEP_TIME_PROJECTION_ANTINODES)), this.repulsionRadius, this.repulsionBlur);
    }

    // closest point methods -------------------------------------------------------------------------------------------------------------------

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
