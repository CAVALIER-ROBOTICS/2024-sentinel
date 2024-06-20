// package frc.robot.filters;

// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.numbers.*;
// //37 degrees, 24.12 radians
// public class ContinuousPoseSTDUpdator {
//     static double xSum, ySum, degSum;
//     static int numEntries = 0;
//     private static double getSTDFromNumber(int num, int sum) {
//         return Math.sqrt(Math.pow((num - (sum / numEntries)), 2) / numEntries);
//     }

//     public void update(Pose2d pose) {
//         xSum += pose.getX();
//         ySum += pose.getY();
//         degSum += pose.getRotation().getDegrees();
//         numEntries++;
//     }
//     public static double[] getSTDs(Pose2d position) { //lmao
//         if(numEntries > 0) {
            
//         }
//     }
// }
