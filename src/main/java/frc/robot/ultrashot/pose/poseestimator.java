package frc.robot.ultrashot.pose;

import frc.robot.ultrashot.Point2D;

public class poseestimator {

    private double fovX, fovY;

    private boolean isRedSide;

    private double percentX, percentY, robotTheta;
    private Point2D camera, robot;

    private double LLforward = 0.34;
    private double LLright = 0;
    private double LLup = 0.1;

    private double LLpitch = 20; // deg

    private Point2D speakerBlue = new Point2D(0, 218.42 * 0.0254);
    private Point2D speakerRed = new Point2D(651.25 * 0.0254, 218.42 * 0.0254);

    public poseestimator() {
        this(0.0, 0.0);
    }

    public poseestimator(double fovX, double fovY) {
        this.fovX = fovX;
        this.fovY = fovY;
    }

    public void configure(double fovX, double fovY) {
        this.fovX = fovX;
        this.fovY = fovY;
    }

    public void update(double percentX, double percentY, double robotTheta) {
        this.percentX = percentX;
        this.percentY = percentY;
        this.robotTheta = robotTheta;
        calculateRobotPosition();
    }

    private void calculateRobotPosition() {
        double thetaX = (percentX - 0.5) * fovX;
        double thetaY = Math.toRadians(LLpitch) - (percentY - 0.5) * fovY;
        double z0 = 57.13 * 0.0254 - LLup;
        double x0 = z0 / Math.tan(thetaY);
        double y0 = x0 * Math.tan(-thetaX);
        double cos = Math.cos(robotTheta);
        double sin = Math.sin(robotTheta);
        camera = new Point2D(
            (-x0)*cos - (-y0)*sin,
            (-y0)*cos + (-x0)*sin
        );
        robot = Point2D.subtract(
            camera,
            new Point2D(
                LLforward*cos - LLright*sin,
                LLright*cos + LLforward*sin
            )
        );
        if (isRedSide) {
            robot = Point2D.subtract(
                speakerRed,
                robot
            );
        }
        else {
            robot = Point2D.add(
                speakerBlue,
                robot
            );
        }
    }

    public void setAlliance(boolean isRedSide) {
        this.isRedSide = isRedSide;
    }

    public double getRobotX() {
        return this.robot.getX();
    }

    public double getRobotY() {
        return this.robot.getY();
    }

    public double getCameraX() {
        return this.camera.getX();
    }

    public double getCameraY() {
        return this.camera.getY();
    }

}