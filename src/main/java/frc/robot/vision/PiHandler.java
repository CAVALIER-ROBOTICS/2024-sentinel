package frc.robot.vision;

import com.jcraft.jsch.JSch;
import com.jcraft.jsch.JSchException;
import com.jcraft.jsch.Session;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class PiHandler {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("targeting");

    static String user = "cavbots";
    static String password = "qobotics";
    static String host = "10.74.92.76";
    static int port = 22;

    public static void initialize() {
        JSch jsch = new JSch();
        try {
            Session session = jsch.getSession(user, host, port);
            session.setPassword(password);
            session.setConfig("StrictHostKeyChecking", "no");
            System.out.println("Connecting to pi...");
            session.connect();
            System.out.println("Logged in over SSH");
        } catch(JSchException e) {
            System.out.println("SSH fail :(");
            e.printStackTrace();
        }
    }

    public static double[] stringToDoubleArray(String raw) {
        String[] arr = raw.split(" ");
        double[] doubleArr = new double[arr.length];
        for(int i = 0; i < arr.length; i++) {
            doubleArr[i] = Double.valueOf(arr[i]);
        }

        return doubleArr;
    }

    private static String getNoteXRaw() {
        return table.getEntry(Constants.PiConstants.noteX).getString("");
    }

    private static String getNoteYRaw() {
        return table.getEntry(Constants.PiConstants.noteY).getString("");
    }

    private static String getBotXRaw() {
        return table.getEntry(Constants.PiConstants.noteY).getString("");
    }

    private static String getBotYRaw() {
        return table.getEntry(Constants.PiConstants.noteY).getString("");
    }

    public static VisionTarget[] getNotePositionsOnscreen() {
        double[] nx = stringToDoubleArray(getNoteXRaw());
        double[] ny = stringToDoubleArray(getNoteYRaw());
  
        VisionTarget[] targets = new VisionTarget[nx.length];
        
        for(int i = 0; i < nx.length; i++) {
            targets[i] = new VisionTarget(nx[i], ny[i]);
        }

        return targets;
    }

    public static VisionTarget[] getBotPositionsOnscreen() {
        double[] nx = stringToDoubleArray(getBotXRaw());
        double[] ny = stringToDoubleArray(getBotYRaw());
  
        VisionTarget[] targets = new VisionTarget[nx.length];
        
        for(int i = 0; i < nx.length; i++) {
            targets[i] = new VisionTarget(nx[i], ny[i]);
        }

        return targets;
    }
}
