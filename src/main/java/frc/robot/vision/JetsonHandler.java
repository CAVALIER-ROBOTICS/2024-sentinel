package frc.robot.vision;

import java.io.IOException;
import java.io.OutputStream;
import com.jcraft.jsch.Channel;
import com.jcraft.jsch.JSch;
import com.jcraft.jsch.JSchException;
import com.jcraft.jsch.Session;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class JetsonHandler {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("vslam");

    static String user = "cavbots";
    static String password = "qobotics";
    static String host = "10.74.92.45";
    static int port = 22;
    Translation2d initialPosition;

    public JetsonHandler(Translation2d initialPosition) {
        this.initialPosition = initialPosition;
        initialize();
    }

    // public void updateInitialPose() {
    //     Pose2d tagPose = getNVIDIAPose(new Rotation2d());
    //     Pose2d slamPose = getVSLAMPose(new Rotation2d());
    //     double xOffset = slamPose.getX() - tagPose.getX();
    //     double yOffset = slamPose.getY() - tagPose.getY();
    //     initialPosition = new Translation2d(initialPosition.getX() - xOffset, initialPosition.getY() - yOffset);
    // }

    public double getStartingPointRelativeX() {
        return table.getEntry(Constants.JetsonConstants.vSlamX).getDouble(-1);
    }

    public double getStartingPointRelativeY() {
        return table.getEntry(Constants.JetsonConstants.vSlamY).getDouble(-1);
    }

    public Boolean isTagging() {
        if (table.getEntry(Constants.JetsonConstants.tagX).isValid() || table.getEntry(Constants.JetsonConstants.tagY).isValid())
        {return false;} 
        else if (table.getEntry(Constants.JetsonConstants.tagX).getDouble(0) > 0 || table.getEntry(Constants.JetsonConstants.tagY).getDouble(0) > 0) 
        {return true;}
        else {return false;}
    }

    public Boolean isSlamming() {
        if (table.getEntry(Constants.JetsonConstants.vSlamX).isValid() || table.getEntry(Constants.JetsonConstants.vSlamY).isValid())
        {return false;} 
        return true;
    }

    public Pose2d getVSLAMPose(Rotation2d botAngle) {
        double x = initialPosition.getX() + getStartingPointRelativeX();
        double y = initialPosition.getY() + getStartingPointRelativeY();

        return new Pose2d(x, y, botAngle);
    }

    public Pose2d getNVIDIAPose(Translation2d initialXY, Rotation2d botAngle) {
        double x = table.getEntry(Constants.JetsonConstants.tagX).getDouble(0);
        double y = table.getEntry(Constants.JetsonConstants.tagY).getDouble(0);

        return new Pose2d(x + initialXY.getX(), y + initialXY.getY(), botAngle);
    }

    public Session getSession() {
        JSch jSch = new JSch();
        try {
            Session s = jSch.getSession(user, host, port);
            s.setConfig("StrictHostKeyChecking", "no");
            s.setConfig("PreferredAuthentications", "password");
            s.setPassword(password);
            return s;
        } catch(JSchException e) {
            System.out.println("Unable to get session");
            System.out.println(e.getMessage());
            return null;
        }
    }

    private void sendCommand(Session s, String command) throws JSchException, IOException {
        Channel c = s.openChannel("shell");
        OutputStream o = c.getOutputStream();
        c.connect();
        o.write((command + "\r\n").getBytes());
    }

    // private void sendCommandExec(Session s, String command) throws JSchException, IOException, InterruptedException {
    //     Channel c = s.openChannel("exec");
    //     ChannelExec exec = (ChannelExec) c;
    //     exec.setCommand(command);
    //     exec.setErrStream(System.err);
    //     exec.setOutputStream(System.out);
    //     c.connect();
    // }

    private void initialize() {
        System.out.println("Initialization ran");
        Session s = getSession();
        try {
            s.connect();
            sendCommand(s, "ifconfig");
            System.out.println("It did indeed run...");
        } catch(Exception e) {
            System.out.println("There was an exception?");
            System.out.println(e.getMessage());
        }
    }
}