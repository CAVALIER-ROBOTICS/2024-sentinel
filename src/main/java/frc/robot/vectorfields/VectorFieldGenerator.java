package frc.robot.vectorfields;

import frc.robot.ultrashot.Point2D;

public class VectorFieldGenerator {

    private double attraction, repulsion;
    private Point2D robot, target, velocity;
    private Point2D[] antiNodes;

    public VectorFieldGenerator() {
        this.attraction = 0.0;
        this.repulsion = 0.0;
        this.robot = new Point2D();
        this.target = new Point2D();
        this.velocity = new Point2D();
        this.antiNodes = new Point2D[3];
    }

    public double getAttraction() {
        return this.attraction;
    }

    public double getRepulsion() {
        return this.repulsion;
    }

    public Point2D getRobot() {
        return this.robot;
    }

    public Point2D getTarget() {
        return this.target;
    }

    public Point2D getVelocity() {
        return this.velocity;
    }

    public Point2D[] getAntiNodes() {
        return this.antiNodes;
    }

    public void setAttraction(double attraction) {
        this.attraction = attraction;
    }

    public void setRepulsion(double repulsion) {
        this.repulsion = repulsion;
    }

    public void setRobot(Point2D robot) {
        this.robot = robot;
    }

    public void setTarget(Point2D target) {
        this.target = target;
    }

    public void setVelocity(Point2D velocity) {
        this.velocity = velocity;
    }

    public void setAntiNodes(Point2D[] antiNodes) {
        this.antiNodes = antiNodes;
    }

    public void configure(double attraction, double repulsion, Point2D target, Point2D[] antinodes) {
        this.attraction = attraction;
        this.repulsion = repulsion;
        this.target = target;
        this.antiNodes = antinodes;
    }

    public void update(Point2D robot) {
        setRobot(robot);
        calculateVelocity();
        // velocity.normalize();
    }

    private void calculateVelocity() {
        Point2D attractor = createAttractor();
        Point2D[] repulsors = {
            createRepulsor(0),
            createRepulsor(1),
            createRepulsor(2)
        };
        velocity = Point2D.add(velocity, attractor);
        for (int i = 0; i < 3; i++) {
            velocity = Point2D.add(velocity, repulsors[i]);
        }
    }

    private Point2D createAttractor() {
        return Point2D.scalar(Point2D.difference(robot, target), attraction / Point2D.difference(robot, target).getHypot());
    }

    private Point2D createRepulsor(int index) {
        return Point2D.scalar(Point2D.difference(antiNodes[index], robot), repulsion / Math.pow(Point2D.difference(antiNodes[index], robot).getHypot(), 2));
    }
    
}
