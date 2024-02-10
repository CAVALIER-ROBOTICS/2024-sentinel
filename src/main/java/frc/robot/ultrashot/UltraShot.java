package frc.robot.ultrashot;

public class UltraShot {

    private Point3D robot, robotVelocity, target;

    public UltraShot() {
        this.robot = new Point3D();
        this.robotVelocity = new Point3D();
        this.target = new Point3D();
    }

    public Point3D getRobotPos() {
        return this.robot;
    }

    public Point3D getRobotVelocity() {
        return this.robotVelocity;
    }

    public Point3D getTargetPos() {
        return this.target;
    }

    public void setRobotPos(Point3D robot) {
        this.robot = robot;
    }

    public void setRobotVelocity(Point3D robotVelocity) {
        this.robotVelocity = robotVelocity;
    }

    public void setTargetPos(Point3D target) {
        this.target = target;
    }

    public Point2D qoboticsUltimatum() {
        double thetaS = Point3D.difference(robot, target).getAngle();
        double thetaV = robotVelocity.getAngle();
        double phiGuess = parabolicAngles(robot, target);
        double thetaGuess = robotVelocity.getHypot() * Math.sin(thetaS - thetaV);

        Point2D initialGuess = new Point2D(0.9 * phiGuess, 0.08 * thetaGuess);

        Point2D angles = slingshot(initialGuess);

        angles.setY(angles.getY() + thetaS);

        return angles;
    }

    private double parabolicAngles(Point3D robotPos, Point3D speakerPos) {
        Point3D difference = Point3D.difference(robotPos, speakerPos);
        double d = Math.hypot(difference.getX(), difference.getY()) - UltrashotConstants.shooterAxisInRobotSpace.getX();
        double h = difference.getZ() - UltrashotConstants.shooterAxisInRobotSpace.getZ();
        double g = UltrashotConstants.localGravity;
        double v = UltrashotConstants.shooterVelocity;
        double v2 = v*v;
        double shooterAngle = Math.atan((v2 - Math.sqrt(v2 * v2 - g*(g*d*d + 2*h*v2))) / (g * d));
        return shooterAngle;
    }

    private double T(double x, double y, boolean isType2) {
        Point3D s = target;
        Point3D r = robot;
        Point3D rv = robotVelocity;
        double l = UltrashotConstants.shooterLength;
        double v0 = UltrashotConstants.shooterVelocity;
        double cosX = Math.cos(x);
        double sinX = Math.sin(x);
        double thetaS = Point3D.difference(r, s).getAngle();
        double thetaV = rv.getAngle();
        Point3D a0 = UltrashotConstants.shooterAxisInRobotSpace;
        Point3D arm = new Point3D(l*cosX, 0, l*sinX);
        Point3D f = toRobotSpace(Point3D.add(a0, arm), thetaS + y);
        Point3D k = Point3D.difference(f, s);
        double thetaK = k.getAngle();
        double hv = rv.getHypot();
        if (!isType2) {
            return hv*Math.sin(thetaK - thetaV) - v0*cosX*Math.sin(thetaS + y - thetaK);
        }
        else {
            double g = UltrashotConstants.localGravity;
            double b = UltrashotConstants.airDrag;
            double term = (k.getHypot()) / (v0*cosX*Math.cos(thetaK - thetaS - y) + hv*Math.cos(thetaK - thetaV));
            return (v0*sinX + g/b)*term + (g/(b*b))*Math.log(1 - b*term) - s.getZ() + a0.getZ() + l*sinX;
        }
    }

    private Point2D slingshot(Point2D p) {
        Point2D nt = nt(p);
        Point2D nt2 = nt2(p);

        Point2D result = new Point2D(sx(p, nt, nt2), sy(p, nt, nt2));

        return result;
    }

    private double sx(Point2D p0, Point2D pa, Point2D pb) {
        double x0 = p0.getX();
        double xa = pa.getX();
        double xb = pb.getX();
        double y0 = p0.getY();
        double ya = pa.getY();
        double yb = pb.getY();
        double aSlope = (xa-x0) / (ya-y0);
        double bSlope = (xb-x0) / (yb-y0);

        return (xb*bSlope - xa*aSlope + yb - ya) / (bSlope - aSlope);
    }

    private double sy(Point2D p0, Point2D pa, Point2D pb) {
        double x0 = p0.getX();
        double xa = pa.getX();
        double xb = pb.getX();
        double y0 = p0.getY();
        double ya = pa.getY();
        double yb = pb.getY();
        double aSlope = (ya-y0) / (xa-x0);
        double bSlope = (yb-y0) / (xb-x0);

        return (yb*bSlope - ya*aSlope + xb - xa) / (bSlope - aSlope);
    }

    private Point2D nt(Point2D p) {
        double x = p.getX();
        double y = p.getY();
        double f = T(x, y, false);
        double dx = 1000*(T(x+0.001, y, false)-f);
        double dy = 1000*(T(x, y+0.001, false)-f);
        double nt0 = f / (dx*dx + dy*dy);
        double ntx = x - dx*nt0;
        double nty = y - dy*nt0;

        Point2D result = new Point2D(ntx, nty);

        return result;
    }

    private Point2D nt2(Point2D p) {
        double x = p.getX();
        double y = p.getY();
        double f = T(x, y, true);
        double dx = 1000*(T(x+0.001, y, true)-f);
        double dy = 1000*(T(x, y+0.001, true)-f);
        double nt0 = f / (dx*dx + dy*dy);
        double ntx = x - dx*nt0;
        double nty = y - dy*nt0;

        Point2D result = new Point2D(ntx, nty);

        return result;
    }

    private Point3D toRobotSpace(Point3D point, double robotAngle) {
        Point3D change = new Point3D(
            point.getX()*Math.cos(robotAngle) - point.getY()*Math.sin(robotAngle),
            point.getY()*Math.cos(robotAngle) + point.getX()*Math.sin(robotAngle),
            point.getZ()
        );
        return Point3D.add(change, robot);
    }

}