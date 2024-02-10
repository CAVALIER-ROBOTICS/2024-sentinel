package frc.robot.ultrashot;

public class Point3D {
    
    private double x, y, z;

    public Point3D() {
        this.x = 0.0;
        this.y = 0.0;
        this.z = 0.0;
    }

    public Point3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public double getZ() {
        return this.z;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setZ(double z) {
        this.z = z;
    }


    public static Point3D difference(Point3D startPoint, Point3D endPoint) {
        Point3D difference = new Point3D(endPoint.getX() - startPoint.getX(), endPoint.getY() - startPoint.getY(), endPoint.getZ() - startPoint.getZ());
        return difference;
    }

    public static Point3D add(Point3D point1, Point3D point2) {
        Point3D sum = new Point3D(point1.getX() + point2.getX(), point1.getY() + point2.getY(), point1.getZ() + point2.getZ());
        return sum;
    }

    public double getAngle() {
        return Math.atan2(this.y, this.x);
    }

    public double getHypot() {
        return Math.hypot(this.x, this.y);
    }

}
