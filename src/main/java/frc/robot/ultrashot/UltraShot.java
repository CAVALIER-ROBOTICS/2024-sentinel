package frc.robot.ultrashot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class UltraShot {

    private Point3D robot, axis, velocity, target; // Note that the axis should have 0 y-component and the velocity should have 0 z-component
    private AngleStates states;
    private double shooterLength, shooterSpeed, localGravity, airDrag, settleTime;

    public UltraShot() {
        this.robot = new Point3D();
        this.axis = new Point3D();
        this.velocity = new Point3D();
        this.target = new Point3D();
        this.states = new AngleStates();
        this.shooterLength = 0.0;
        this.shooterSpeed = 0.0;
        this.localGravity = 0.0;
        this.airDrag = 0.0;
        this.settleTime = 0.0;
    }

    // Get Methods

    public Point3D getRobot() {
        return this.robot;
    }

    public Point3D getAxis() {
        return this.axis;
    }

    public Point3D getVelocity() {
        return this.velocity;
    }

    public Point3D getTarget() {
        return this.target;
    }

    public AngleStates getAngleStates() {
        return this.states;
    }

    public double getShooterLength() {
        return this.shooterLength;
    }

    public double getShooterVelocity() {
        return this.shooterSpeed;
    }

    public double getLocalGravity() {
        return this.localGravity;
    }

    public double getAirDrag() {
        return this.airDrag;
    }

    public double getTheta() {
        return this.states.getTheta();
    }

    public double getOmega() {
        return this.states.getOmega();
    }

    public double getPhi() {
        return this.states.getPhi();
    }

    public double getPsi() {
        return this.states.getPsi();
    }

    public double getProcessTime() {
        return this.settleTime;
    }

    // Set Methods

    public void setRobot(Point3D robot) {
        this.robot = robot;
    }

    public void setAxis(Point3D axis) {
        this.axis = axis;
    }

    public void setVelocity(Point3D velocity) {
        this.velocity = velocity;
    }

    public void setTarget(Point3D target) {
        this.target = target;
    }

    public void setAngleStates(AngleStates states) {
        this.states = states;
    }

    public void setShooterLength(double shooterLength) {
        this.shooterLength = shooterLength;
    }

    public void setShooterSpeed(double shooterSpeed) {
        this.shooterSpeed = shooterSpeed;
    }

    public void setLocalGravity(double localGravity) {
        this.localGravity = localGravity;
    }

    public void setAirDrag(double airDrag) {
        this.airDrag = airDrag;
    }

    public void setSettleTime(double settleTime) {
        this.settleTime = settleTime;
    }

    public void configure(Point3D robot, Point3D axis, Point3D velocity, Point3D target, AngleStates states, double shooterLength, double shooterSpeed, double localGravity, double airDrag, double settleTime) {
        setRobot(robot);
        setAxis(axis);
        setVelocity(velocity);
        setTarget(target);
        setAngleStates(states);
        setShooterLength(shooterLength);
        setShooterSpeed(shooterSpeed);
        setLocalGravity(localGravity);
        setAirDrag(airDrag);
        setSettleTime(settleTime);
    }

    public void update(Point3D robot, Point3D velocity, Point3D target) {
        setRobot(robot);
        setVelocity(velocity);
        setTarget(target);
    }

    public void update(Pose2d pose, ChassisSpeeds velocity) {
        Point3D pointPose = new Point3D(pose.getX(), pose.getY(), 0);
        Point3D pointVelocity = new Point3D(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, 0);

        update(pointPose, pointVelocity, UltraShotConstants.target);
    }
    // ultimatum() returns the true values of theta and phi
    // omega and psi are zero since it is impractical to shoot while changing the angles
    // use this when you want to shoot

    public void ultimatum() {
        Point3D prediction = Point3D.add(robot, new Point3D(velocity.getX()*settleTime, velocity.getY()*settleTime, velocity.getZ()*settleTime));
        double thetaS = Point3D.difference(prediction, target).getAngle();
        double thetaV = velocity.getAngle();
        parabolicAngle(prediction, target, true);
        parabolicAngle(prediction, target, false);
        states.setTheta(velocity.getHypot() * Math.sin(thetaS - thetaV) * 0.08);
        
        slingShot();

        states.setTheta(states.getTheta() + thetaS);
    }

    // track() returns approximate values of theta, omega, phi, and psi so that a control loop has less error to deal with when ultimatum() is called
    // use this when you want to shoot SOON

    public void track() {
        Point3D prediction = Point3D.add(robot, new Point3D(velocity.getX()*settleTime, velocity.getY()*settleTime, velocity.getZ()*settleTime));
        double thetaS = Point3D.difference(prediction, target).getAngle();
        double thetaV = velocity.getAngle();
        parabolicAngle(prediction, target, true);
        parabolicAngle(prediction, target, false);
        states.setTheta(velocity.getHypot() * Math.sin(thetaS - thetaV) * 0.08 + thetaS);
    }

    private void parabolicAngle(Point3D robot, Point3D target, boolean isFirst) {
        double g = localGravity;
        if (isFirst) {
            double d = Point3D.difference(robot, target).getHypot() - axis.getX();
            double h = target.getZ() - robot.getZ() - axis.getZ();
            states.setPhi( Math.atan((Math.pow(shooterSpeed, 2) - Math.sqrt(Math.pow(shooterSpeed, 4) - g*(g*d*d + 2*h*Math.pow(shooterSpeed, 2)))) / (g * d)));
        }
        else {
            double d = Point3D.difference(robot, target).getHypot() - axis.getX() - shooterLength*Math.cos(states.getPhi());
            double h = target.getZ() - robot.getZ() - axis.getZ() - shooterLength*Math.sin(states.getPhi());
            states.setPhi( Math.atan((Math.pow(shooterSpeed, 2) - Math.sqrt(Math.pow(shooterSpeed, 4) - g*(g*d*d + 2*h*Math.pow(shooterSpeed, 2)))) / (g * d)));
        }
    }

    private void slingShot() {
        AngleStates projection1 = projection1(states);
        AngleStates projection2 = projection2(states);

        double theta0 = states.getTheta();
        double theta1 = projection1.getTheta();
        double theta2 = projection2.getTheta();
        double phi0 = states.getPhi();
        double phi1 = projection1.getPhi();
        double phi2 = projection2.getPhi();

        double thetaSlope1 = (theta1-theta0) / (phi1-phi0);
        double thetaSlope2 = (theta2-theta0) / (phi2-phi0);

        double phiSlope1 = (phi1-phi0) / (theta1-theta0);
        double phiSlope2 = (phi2-phi0) / (theta2-theta0);

        double thetaSlingShot = (theta2*thetaSlope2 - theta1*thetaSlope1 + phi2 - phi1) / (thetaSlope2 - thetaSlope1);
        double phiSlingShot = (phi2*phiSlope2 - phi1*phiSlope1 + theta2 - theta1) / (phiSlope2 - phiSlope1);

        states.setAll(thetaSlingShot, 0.0, phiSlingShot, 0.0);
    }

    private AngleStates projection1(AngleStates states) {
        double theta = states.getTheta();
        double phi = states.getPhi();
        double tValue = T(theta, phi, false);
        double dTheta = 1000 * (T(theta + 0.001, phi, false) - tValue);
        double dPhi = 1000 * (T(theta, phi + 0.001, false) - tValue);
        double n0 = tValue / (dTheta * dTheta + dPhi * dPhi);
        double nTheta = theta - dTheta * n0;
        double nPhi = phi - dPhi * n0;

        return new AngleStates(nTheta, 0.0, nPhi, 0.0);
    }

    private AngleStates projection2(AngleStates states) {
        double theta = states.getTheta();
        double phi = states.getPhi();
        double tValue = T(theta, phi, true);
        double dTheta = 1000 * (T(theta + 0.001, phi, true) - tValue);
        double dPhi = 1000 * (T(theta, phi + 0.001, true) - tValue);
        double n0 = tValue / (dTheta * dTheta + dPhi * dPhi);
        double nTheta = theta - dTheta * n0;
        double nPhi = phi - dPhi * n0;

        return new AngleStates(nTheta, 0.0, nPhi, 0.0);
    }

    private double T(double theta, double phi, boolean isType2) {
        Point3D prediction = Point3D.add(robot, new Point3D(velocity.getX()*settleTime, velocity.getY()*settleTime, velocity.getZ()*settleTime));
        double cosPhi = Math.cos(phi);
        double sinPhi = Math.sin(phi);
        double thetaS = Point3D.difference(prediction, target).getAngle();
        double thetaV = velocity.getAngle();
        double hypotV = velocity.getHypot();
        Point3D freedom = toRobotSpace(Point3D.add(axis, new Point3D(shooterLength*cosPhi, 0, shooterLength*sinPhi)), thetaS + theta);
        Point3D k = Point3D.difference(freedom, target);
        double thetaK = k.getAngle();
        if (!isType2) {
            return hypotV*Math.sin(thetaK - thetaV) - shooterSpeed*cosPhi*Math.sin(thetaS + theta - thetaK);
        }
        else {
            double g = localGravity;
            double b = airDrag;
            double term = (k.getHypot()) / (shooterSpeed*cosPhi*Math.cos(thetaK - thetaS - theta) + hypotV*Math.cos(thetaK - thetaV));
            double value = (shooterSpeed*sinPhi + g/b)*term + (g/(b*b))*Math.log(1 - b*term) - target.getZ() + axis.getZ() + shooterLength*sinPhi;
            return value;
        }
    }

    private Point3D toRobotSpace(Point3D point, double theta) {
        Point3D prediction = Point3D.add(robot, new Point3D(velocity.getX()*settleTime, velocity.getY()*settleTime, velocity.getZ()*settleTime));
        Point3D change = new Point3D(
            point.getX()*Math.cos(theta) - point.getY()*Math.sin(theta),
            point.getY()*Math.cos(theta) + point.getX()*Math.sin(theta),
            point.getZ()
        );
        return Point3D.add(change, prediction);
    }

}