package frc.robot.ultrashot;

public class Point2D {
    
    private double x, y;

    public Point2D() {
        this.x = 0.0;
        this.y = 0.0;
    }

    public Point2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }


    public static Point2D difference(Point2D startPoint, Point2D endPoint) {
        Point2D difference = new Point2D(endPoint.getX() - startPoint.getX(), endPoint.getY() - startPoint.getY());
        return difference;
    }

    public static Point2D subtract(Point2D endPoint, Point2D startPoint) {
        Point2D difference = new Point2D(endPoint.getX() - startPoint.getX(), endPoint.getY() - startPoint.getY());
        return difference;
    }

    public static Point2D add(Point2D point1, Point2D point2) {
        Point2D sum = new Point2D(point1.getX() + point2.getX(), point1.getY() + point2.getY());
        return sum;
    }

    public static Point2D scalar(Point2D point, double scalar) {
        return new Point2D(scalar * point.getX(), scalar * point.getY());
    }

    public void normalize() {
        double h = getHypot();
        this.x /= h;
        this.y /= h;
    }

    public double getAngle() {
        return Math.atan2(this.y, this.x);
    }

    public double getHypot() {
        return Math.hypot(this.x, this.y);
    }

}
