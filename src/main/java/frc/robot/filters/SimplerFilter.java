package frc.robot.filters;

public class SimplerFilter {
    private static int consecutiveTrues = 0;

    private static int consecutiveTruesThresh = 3;

    public static boolean filter(boolean input) {
        if (input) {consecutiveTrues++;} else {consecutiveTrues=0;}
        if (consecutiveTrues >= consecutiveTruesThresh) {return true;} return false;
    }
}
