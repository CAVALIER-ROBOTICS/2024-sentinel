package frc.robot.vectorfields;

import frc.robot.ultrashot.Point3D;

public class NotePoseEstimator {

    private double fovX, fovY;
    private Point3D camera;

    private double percentX, percentY, theta;
    private Point3D robot, note;

    public NotePoseEstimator() {
        this(0.0, 0.0, new Point3D());
    }

    public NotePoseEstimator(double fovX, double fovY, Point3D camera) {
        this.fovX = fovX;
        this.fovY = fovY;
        this.camera = camera;
    }

    public void configure(double fovX, double fovY, Point3D camera) {
        this.fovX = fovX;
        this.fovY = fovY;
        this.camera = camera;
    }

    public void update(double percentX, double percentY, Point3D robot, double theta) {
        this.percentX = percentX;
        this.percentY = percentY;
        this.robot = robot;
        this.theta = theta;
        calculateNotePosition();
    }

    private void calculateNotePosition() {
        double thetaX = (percentX - 0.5) * fovX;
        double thetaY = -(percentY - 0.5) * fovY;
        double z0 = 0.0254 - camera.getZ();
        double x0 = z0 / Math.tan(thetaY);
        double y0 = x0 * Math.tan(-thetaX);
        note = Point3D.add(new Point3D(x0, y0, z0), camera);
        double cosTheta = Math.cos(theta);
        double sinTheta = Math.sin(theta);
        Point3D change = new Point3D(
            note.getX()*cosTheta - note.getY()*sinTheta,
            note.getY()*cosTheta + note.getX()*sinTheta,
            note.getZ()
        );
        note = Point3D.add(change, robot);
    }

    public double getNoteX() {
        return this.note.getX();
    }

    public double getNoteY() {
        return this.note.getY();
    }

    public double getNoteZ() {
        return this.note.getZ();
    }
    
}
