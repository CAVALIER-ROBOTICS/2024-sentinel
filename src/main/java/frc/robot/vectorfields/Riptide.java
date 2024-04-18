package frc.robot.vectorfields;

import frc.robot.ultrashot.Point2D;

public class Riptide {

    private double speed = RiptideConstants.STANDARD_SPEED;
    
    private Point2D robot = new Point2D();
    private Point2D velocity = new Point2D();

    private MobileNode[] mobileNodes = new MobileNode[RiptideConstants.MAXIMUM_NUM_MOBILE_NODES];
    private MobileAntiNode[] mobileAntiNodes = new MobileAntiNode[RiptideConstants.MAXIMUM_NUM_MOBILE_ANTINODES];

    private StaticNode target = new StaticNode(new Point2D(1, 1));

    public Riptide() {
        for (int i = 0; i < this.mobileNodes.length; i++) {
            this.mobileNodes[i] = new MobileNode();
        }
        for (int i = 0; i < mobileAntiNodes.length; i++) {
            this.mobileAntiNodes[i] = new MobileAntiNode();
        }
    }

    // getters -----------------------------------------------------------------------------------------------------------------------------

    public double getSpeed() {
        return this.speed;
    }

    public Point2D getRobot() {
        return this.robot;
    }

    public Point2D getVelocity() {
        return this.velocity;
    }

    public StaticNode getTarget() {
        return this.target;
    }

    // mobile array getters ------------------------------------------------------------------------------------------------------------------


    public MobileNode[] getMobileNodes() {
        return this.mobileNodes;
    }

    public MobileAntiNode[] getMobileAntiNodes() {
        return this.mobileAntiNodes;
    }

    // chassis setpoint getters --------------------------------------------------------------------------------------------------------------

    public double getVX() {
        return this.velocity.getX();
    }

    public double getVY() {
        return this.velocity.getY();
    }

    public double getTheta() {
        return Point2D.subtract(target.getPosition(), robot).getAngle();
    }

    // setters ------------------------------------------------------------------------------------------------------------------------

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public void setRobot(Point2D robot) {
        this.robot = robot;
    }

    public void setTarget(StaticNode target) {
        this.target = target;
    }

    public void setMobileNodes(MobileNode[] mobileNodes) {
        this.mobileNodes = mobileNodes;
    }

    public void setMobileAntiNodes(MobileAntiNode[] mobileAntiNodes) {
        this.mobileAntiNodes = mobileAntiNodes;
    }

    // target setting -------------------------------------------------------------------------------------------------------------------

    public void setTargetSpeakerBlue() {
        this.target = RiptideConstants.SPEAKER_NODE_BLUE;
    }

    public void setTargetSpeakerRed() {
        this.target = RiptideConstants.SPEAKER_NODE_RED;
    }

    public void setTargetAmpBlue() {
        this.target = RiptideConstants.AMP_NODE_BLUE;
    }

    public void setTargetAmpRed() {
        this.target = RiptideConstants.AMP_NODE_RED;
    }

    public void smartSetTargetStatic() {
        Point2D[] nodePositions = new Point2D[RiptideConstants.STARTING_NOTES.length];
        for (int i = 0; i < RiptideConstants.STARTING_NOTES.length; i++) {
            nodePositions[i] = RiptideConstants.STARTING_NOTES[i].getPosition();
        }
        target = RiptideConstants.STARTING_NOTES[robot.closestIndex(nodePositions)];
    }

    public void smartSetTargetMobile() {
        Point2D[] nodePositions = new Point2D[mobileNodes.length];
        for (int i = 0; i < mobileNodes.length; i++) {
            nodePositions[i] = mobileNodes[i].getPosition();
        }
        target = mobileNodes[robot.closestIndex(nodePositions)].getAsStaticNode();
    }

    // update methods ---------------------------------------------------------------------------------------------------------------------

    public void update(Point2D robot, Point2D[] mobileNodes, Point2D[] mobileAntiNodes) {
        //System.out.println(mobileNodes.length + " " + mobileAntiNodes.length);
        setRobot(robot);
        determineNewNotes(mobileNodes);
        determineNewMans(mobileAntiNodes);
        calculateVelocity();
    }

    private void determineNewNotes(Point2D[] positions) {
        int surplus = positions.length;
        for (int i = 0; i < this.mobileNodes.length; i++) {
            if (!this.mobileNodes[i].getIsFaded()) {
                if (this.mobileNodes[i].closestDistance(positions) < RiptideConstants.STEP_TOLERANCE_NODES) {
                    this.mobileNodes[i].update(this.mobileNodes[i].closest(positions), true);
                    positions[this.mobileNodes[i].closestIndex(positions)].blastOff();
                    surplus--;
                }
                else {
                    this.mobileNodes[i].update(null, false);
                }
            }
        }
        while (surplus > 0) {
            for (int i = 0; i < this.mobileNodes.length; i++) {
                if (this.mobileNodes[i].getIsFaded()) {
                    this.mobileNodes[i].update(this.mobileNodes[i].closest(positions), true);
                    positions[this.mobileNodes[i].closestIndex(positions)].blastOff();
                    surplus--;
                    break;
                }
            }
        }
    }

    private void determineNewMans(Point2D[] positions) {
        int surplus = positions.length;
        for (int i = 0; i < this.mobileAntiNodes.length; i++) {
            System.out.println("looks at man" + i);
            System.out.println(this.mobileAntiNodes[i].getIsFaded());
            if (!this.mobileAntiNodes[i].getIsFaded()) {
                if (this.mobileAntiNodes[i].closestDistance(positions) < RiptideConstants.STEP_TOLERANCE_ANTINODES) {
                    this.mobileAntiNodes[i].update(this.mobileAntiNodes[i].closest(positions), true);
                    positions[this.mobileAntiNodes[i].closestIndex(positions)].blastOff();
                    surplus--;
                }
                else {
                    this.mobileAntiNodes[i].update(null, false);
                }
            }
        }
        System.out.println("has the number" + surplus);
        while (surplus > 0) {
            for (int i = 0; i < this.mobileAntiNodes.length; i++) {
                if (this.mobileAntiNodes[i].getIsFaded()) {
                    this.mobileAntiNodes[i].update(this.mobileAntiNodes[i].closest(positions), true);
                    positions[this.mobileAntiNodes[i].closestIndex(positions)].blastOff();
                    surplus--;
                    break;
                }
            }
        }
    }

    private void calculateVelocity() {

        System.out.println(velocity.getX());

        velocity = Point2D.add(velocity, target.getVector(robot));

        System.out.println(velocity.getX());

        for (int i = 0; i < RiptideConstants.FIELD_ELEMENTS.length; i++) {
            System.out.println(i + " " + velocity.getX());
            velocity = Point2D.subtract(velocity, RiptideConstants.FIELD_ELEMENTS[i].getVector(robot));
            System.out.println(i + " " + velocity.getX());
        }

        System.out.println(velocity.getX());

        for (int i = 0; i < mobileAntiNodes.length; i++) {
            System.out.println("man" + i + " " + velocity.getX());
            velocity = Point2D.subtract(velocity, mobileAntiNodes[i].getVector(robot));
            System.out.println("man" + i + " " + velocity.getX());
        }

        System.out.println(velocity.getX());

        velocity.normalize(speed);

        System.out.println(velocity.getX());

    }

}