

// public class Main {

//     public static void main(String[] args) {

//         UltraShot shooter = new UltraShot();

//         shooter.configure(
//             UltraShotConstants.robot,
//             UltraShotConstants.axis,
//             UltraShotConstants.velocity,
//             UltraShotConstants.target,
//             UltraShotConstants.states,
//             UltraShotConstants.shooterLength,
//             UltraShotConstants.shooterSpeed,
//             UltraShotConstants.localGravity,
//             UltraShotConstants.airDrag,
//             UltraShotConstants.settleTime
//         );
        
//         shooter.update(UltraShotConstants.robot, UltraShotConstants.velocity, UltraShotConstants.target);
//         shooter.track();
//         printAngles(shooter);

//         shooter.update(UltraShotConstants.robot, UltraShotConstants.velocity, UltraShotConstants.target);
//         shooter.track();
//         printAngles(shooter);

//         shooter.update(UltraShotConstants.robot, UltraShotConstants.velocity, UltraShotConstants.target);
//         shooter.track();
//         printAngles(shooter);

//         shooter.update(UltraShotConstants.robot, UltraShotConstants.velocity, UltraShotConstants.target);
//         shooter.ultimatum();
//         printAngles(shooter);
        
//     }

//     public static void printAngles(UltraShot shooter) {
//         System.out.println("Theta: " + shooter.getTheta() + " Omega: " + shooter.getOmega() + " Phi: " + shooter.getPhi() + " Psi: " + shooter.getPsi());
//     }

// }