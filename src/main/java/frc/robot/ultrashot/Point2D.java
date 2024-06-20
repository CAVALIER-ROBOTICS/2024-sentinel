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

    public static Point2D add(Point2D point1, Point2D point2) {
        return new Point2D(point1.getX() + point2.getX(), point1.getY() + point2.getY());
    }

    public static Point2D subtract(Point2D point1, Point2D point2) {
        return new Point2D(point1.getX() - point2.getX(), point1.getY() - point2.getY());
    }

    public static Point2D scalar(Point2D point, double scalar) {
        return new Point2D(scalar * point.getX(), scalar * point.getY());
    }

    public void normalize(double magnitude) {
        double h = getHypot();
        this.x /= h;
        this.x *= magnitude;
        this.y /= h;
        this.y *= magnitude;
    }

    public void blastOff() {
        this.x = -99999999999.9;
        this.y = -99999999999.9;
    }

    public double getAngle() {
        return Math.atan2(this.y, this.x);
    }

    public double getHypot() {
        return Math.hypot(this.x, this.y);
    }

    public double distanceTo(Point2D point) {
        return difference(this, point).getHypot();
    }

    public Point2D closest(Point2D[] points) {
        Point2D closestPoint = points[0];
        for (int i = 0; i < points.length - 1; i++) {
            double distance1 = distanceTo(points[i]);
            double distance2 = distanceTo(points[i + 1]);
            if (distance2 <= distance1) {
                closestPoint = points[i + 1];
            }
        }
        return closestPoint;
    }

    public int closestIndex(Point2D[] points) {
        int closestIndex = 0;
        for (int i = 0; i < points.length - 1; i++) {
            double distance1 = distanceTo(points[i]);
            double distance2 = distanceTo(points[i + 1]);
            if (distance2 <= distance1) {
                closestIndex = i + 1;
            }
        }
        return closestIndex;
    }

    public double closestDistance(Point2D[] points) {
        double closestDistance = 99999.9;
        for (int i = 1; i < points.length; i++) {
            double distance = distanceTo(points[i]);
            if (distance <= closestDistance) {
                closestDistance = distance;
            }
        }
        return closestDistance;
    }

}
