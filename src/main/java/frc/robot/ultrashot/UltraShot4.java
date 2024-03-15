package frc.robot.ultrashot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class UltraShot4 {

    private Point3D robot, shooterAxis, velocity, acceleration, target;
    private Point3D positionPrediction, velocityPrediction;
    private Point3D[] velocityHistory;
    private double theta, omega, phi, psi;
    private double shooterLength, flyWheelVelocity, flyWheelAcceleration, localGravity, airDrag, freedomTime;
    private int numFramesRobotHistory;
    private double[] timeHistory, flyWheelVelocityHistory;

    public UltraShot4() {
        this.robot = new Point3D();
        this.shooterAxis = UltraShotConstants.POINT_3D_SHOOTER_AXIS;
        this.velocity = new Point3D();
        this.acceleration = new Point3D();
        this.target = new Point3D();
        this.positionPrediction = new Point3D();
        this.velocityPrediction = new Point3D();
        this.velocityHistory = new Point3D[UltraShotConstants.NUM_FRAMES_ROBOT_HISTORY];
        for (int i = 0; i < velocityHistory.length; i++) {
            this.velocityHistory[i] = new Point3D();
        }
        this.theta = 0.0;
        this.omega = 0.0;
        this.phi = 0.0;
        this.psi = 0.0;
        this.shooterLength = UltraShotConstants.SHOOTER_LENGTH;
        this.flyWheelVelocity = 0.0;
        this.flyWheelAcceleration = 0.0;
        this.localGravity = UltraShotConstants.LOCAL_GRAVITY;
        this.airDrag = UltraShotConstants.AIR_DRAG;
        this.freedomTime = UltraShotConstants.FREEDOM_TIME;
        this.numFramesRobotHistory = 0;
        this.timeHistory = new double[UltraShotConstants.NUM_FRAMES_ROBOT_HISTORY];
        this.flyWheelVelocityHistory = new double[UltraShotConstants.NUM_FRAMES_ROBOT_HISTORY];
    }

    public AngleStates getAngleStates() {
        return new AngleStates(getTheta(), getOmega(), getPhi(), getPsi());
    }

    // Get Methods

    public double getTheta() {
        return this.theta;
    }

    public double getOmega() {
        return this.omega;
    }

    public double getPhi() {
        return this.phi;
    }

    public double getPsi() {
        return this.psi;
    }

    // Set Methods

    public void setTargetSpeakerBlue() {
        this.target = UltraShotConstants.POINT_3D_SPEAKER_BLUE;
    }

    public void setTargetSpeakerRed() {
        this.target = UltraShotConstants.POINT_3D_SPEAKER_RED;
    }

    public void update(Pose2d robotPose, ChassisSpeeds velo, double speed, double timeSinceLastUpdate) {
        Point3D pose = new Point3D(robotPose.getX(), robotPose.getY(), 0);
        Point3D velocity = new Point3D(velo.vxMetersPerSecond, velo.vyMetersPerSecond, 0);
        update(pose, velocity, speed, timeSinceLastUpdate);
    }

    public void update(Point3D robotUpdate, Point3D velocityUpdate, double flyWheelVelocityUpdate, double timeSinceLastUpdate) {

        if (velocityUpdate.isAllZeroes()) {
            velocityUpdate = new Point3D(0.000001, 0, 0);
        }
        updateRobotHistory(timeSinceLastUpdate, flyWheelVelocityUpdate, velocityUpdate);
        updateAccelerations();

        this.robot = robotUpdate;
        this.velocity = velocityUpdate;
        this.flyWheelVelocity = flyWheelVelocityUpdate;

        ultimatum();

        double currentTheta = this.theta;
        double currentPhi = this.phi;

        this.robot.add(Point3D.scalar(velocity, 0.00001));
        this.flyWheelVelocity += this.flyWheelAcceleration * 0.00001;

        ultimatum();

        double nextTheta = this.theta;
        double nextPhi = this.phi;

        this.theta = currentTheta;
        this.omega = (nextTheta - currentTheta) / 0.00001;
        this.phi = currentPhi;
        this.psi = (nextPhi - currentPhi) / 0.00001;

    }

    private void updateRobotHistory(double timeSinceLastUpdate, double flyWheelVelocityUpdate, Point3D velocityUpdate) {
        if (this.numFramesRobotHistory < UltraShotConstants.NUM_FRAMES_ROBOT_HISTORY) {
            this.numFramesRobotHistory++;
        }
        for (int i = 0; i < UltraShotConstants.NUM_FRAMES_ROBOT_HISTORY; i++) {
            this.timeHistory[i] -= timeSinceLastUpdate;
        }
        for (int i = UltraShotConstants.NUM_FRAMES_ROBOT_HISTORY - 1; i > 0; i--) {
            this.timeHistory[i] = this.timeHistory[i - 1];
            this.flyWheelVelocityHistory[i] = this.flyWheelVelocityHistory[i - 1];
            this.velocityHistory[i] = this.velocityHistory[i - 1];
        }
        this.timeHistory[0] = 0.0;
        this.flyWheelVelocityHistory[0] = flyWheelVelocityUpdate;
        this.velocityHistory[0] = velocityUpdate;
    }

    private void updateAccelerations() {
        this.flyWheelAcceleration = linRegFlyWheelAcceleration();
        this.acceleration = linRegAcceleration();
    }

    private Point3D linRegAcceleration() {
        int m = this.numFramesRobotHistory;
        if (m == 1) {
            return new Point3D();
        }
        else {
            double[] X = new double[m];
            double[] Y = new double[m];
            double[] T = new double[m];
            double sumX = 0;
            double sumY = 0;
            double sumT = 0;
            for (int i = 0; i < m; i++) {
                X[i] = this.velocityHistory[i].getX();
                Y[i] = this.velocityHistory[i].getY();
                T[i] = this.timeHistory[i];
                sumX += X[i];
                sumY += Y[i];
                sumT += T[i];
            }
            double sumNumX = 0;
            double sumNumY = 0;
            double sumDen = 0;
            for (int i = 0; i < m; i++) {
                sumNumX += T[i] * (sumX - m * X[i]);
                sumNumY += T[i] * (sumY - m * Y[i]);
                sumDen += T[i] * (sumT - m * T[i]);
            }
            return Point3D.scalar(new Point3D(sumNumX, sumNumY, 0), 1 / sumDen);
        }
    }

    private double linRegFlyWheelAcceleration() {
        int m = this.numFramesRobotHistory;
        if (m == 1) {
            return 0.0;
        }
        else {
            double[] X = new double[m];
            double[] T = new double[m];
            double sumX = 0;
            double sumT = 0;
            for (int i = 0; i < m; i++) {
                X[i] = this.flyWheelVelocityHistory[i];
                T[i] = this.timeHistory[i];
                sumX += X[i];
                sumT += T[i];
            }
            double sumNum = 0;
            double sumDen = 0;
            for (int i = 0; i < m; i++) {
                sumNum += T[i] * (sumX - m * X[i]);
                sumDen += T[i] * (sumT - m * T[i]);
            }
            return sumNum / sumDen;
        }
    }

    private void ultimatum() {

        robotPrediction();

        double thetaS = Point3D.difference(this.positionPrediction, this.target).getAngle();
        double thetaV = this.velocityPrediction.getAngle();
        parabolicAngle(this.positionPrediction, this.target);
        this.theta = this.velocityPrediction.getHypot() * Math.sin(thetaS - thetaV) * 0.08;
        
        slingShot();

        if (Double.isNaN(this.theta)) {
            this.theta = thetaS;
        }
        else {
            this.theta += thetaS;
        }
    }

    private void robotPrediction() {

        this.positionPrediction.copy(this.robot);
        this.positionPrediction.add(Point3D.scalar(this.velocity, this.freedomTime));
        this.positionPrediction.add(Point3D.scalar(this.acceleration, Math.pow(this.freedomTime, 2)));

        this.velocityPrediction.copy(this.velocity);
        this.velocityPrediction.add(Point3D.scalar(this.acceleration, this.freedomTime));

        if (velocityPrediction.getHypot() > UltraShotConstants.MAX_ROBOT_SPEED) {
            System.out.println("flag");
            double ax = this.acceleration.getX();
            double ay = this.acceleration.getY();
            double bx = this.velocity.getX();
            double by = this.velocity.getY();
            double vMax = UltraShotConstants.MAX_ROBOT_SPEED;
            double aH = ax*ax + ay*ay;
            double bH = bx*bx + by*by;
            double abH = ax*bx + ay*by;
            double tMax = (Math.sqrt(abH*abH - aH*(bH - vMax*vMax)) - abH) / (2*aH);
            double tLeft = this.freedomTime - tMax;

            this.positionPrediction.copy(this.robot);
            this.positionPrediction.add(Point3D.scalar(this.velocity, tMax));
            this.positionPrediction.add(Point3D.scalar(this.acceleration, tMax * tMax));

            this.velocityPrediction.copy(this.velocity);
            this.velocityPrediction.add(Point3D.scalar(this.acceleration, tMax));

            this.positionPrediction.add(Point3D.scalar(this.velocityPrediction, tLeft));
        }

    }

    private void parabolicAngle(Point3D robot, Point3D target) {
        double g = this.localGravity;
        double d, h;
        d = Point3D.difference(robot, target).getHypot() - shooterAxis.getX();
        h = target.getZ() - robot.getZ() - shooterAxis.getZ();
        this.phi = Math.atan((Math.pow(flyWheelVelocity, 2) - Math.sqrt(Math.pow(flyWheelVelocity, 4) - g*(g*d*d + 2*h*Math.pow(flyWheelVelocity, 2)))) / (g * d));
        d = Point3D.difference(robot, target).getHypot() - shooterAxis.getX() - shooterLength*Math.cos(this.phi);
        h = target.getZ() - robot.getZ() - shooterAxis.getZ() - shooterLength*Math.sin(this.phi);
        this.phi = Math.atan((Math.pow(flyWheelVelocity, 2) - Math.sqrt(Math.pow(flyWheelVelocity, 4) - g*(g*d*d + 2*h*Math.pow(flyWheelVelocity, 2)))) / (g * d));
    }

    private void slingShot() {

        double tValue1 = T(this.theta, this.phi, false);
        double dTheta1 = 1000 * (T(theta + 0.001, this.phi, false) - tValue1);
        double dPhi1 = 1000 * (T(this.theta, this.phi + 0.001, false) - tValue1);
        double n01 = tValue1 / (dTheta1 * dTheta1 + dPhi1 * dPhi1);
        double theta1 = this.theta - dTheta1 * n01;
        double phi1 = this.phi - dPhi1 * n01;

        double tValue2 = T(this.theta, this.phi, true);
        double dTheta2 = 1000 * (T(this.theta + 0.001, this.phi, true) - tValue2);
        double dPhi2 = 1000 * (T(this.theta, this.phi + 0.001, true) - tValue2);
        double n02 = tValue2 / (dTheta2 * dTheta2 + dPhi2 * dPhi2);
        double theta2 = this.theta - dTheta2 * n02;
        double phi2 = this.phi - dPhi2 * n02;

        double thetaSlope1 = (theta1 - this.theta) / (phi1 - this.phi);
        double thetaSlope2 = (theta2 - this.theta) / (phi2 - this.phi);

        double phiSlope1 = (phi1 - this.phi) / (theta1 - this.theta);
        double phiSlope2 = (phi2 - this.phi) / (theta2 - this.theta);

        this.theta = (theta2*thetaSlope2 - theta1*thetaSlope1 + phi2 - phi1) / (thetaSlope2 - thetaSlope1);
        this.phi = (phi2*phiSlope2 - phi1*phiSlope1 + theta2 - theta1) / (phiSlope2 - phiSlope1);

    }

    private double T(double theta, double phi, boolean isType2) {
        double cosPhi = Math.cos(phi);
        double sinPhi = Math.sin(phi);
        double thetaS = Point3D.difference(this.positionPrediction, this.target).getAngle();
        double thetaV = this.velocityPrediction.getAngle();
        double hypotV = this.velocityPrediction.getHypot();
        Point3D freedom = toRobotSpace(Point3D.add(this.shooterAxis, new Point3D(this.shooterLength*cosPhi, 0, this.shooterLength*sinPhi)), thetaS + theta);
        Point3D k = Point3D.difference(freedom, this.target);
        double thetaK = k.getAngle();
        if (!isType2) {
            return hypotV*Math.sin(thetaK - thetaV) - this.flyWheelVelocity*cosPhi*Math.sin(thetaS + theta - thetaK);
        }
        else {
            double g = localGravity;
            double b = airDrag;
            double term = (k.getHypot()) / (flyWheelVelocity*cosPhi*Math.cos(thetaK - thetaS - theta) + hypotV*Math.cos(thetaK - thetaV));
            double value = (this.flyWheelVelocity*sinPhi + g/b)*term + (g/(b*b))*Math.log(1 - b*term) - this.target.getZ() + this.shooterAxis.getZ() + this.shooterLength*sinPhi;
            return value;
        }
    }

    private Point3D toRobotSpace(Point3D point, double theta) {
        Point3D change = new Point3D(
            point.getX()*Math.cos(theta) - point.getY()*Math.sin(theta),
            point.getY()*Math.cos(theta) + point.getX()*Math.sin(theta),
            point.getZ()
        );
        return Point3D.add(change, this.positionPrediction);
    }

}