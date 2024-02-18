package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auton {

    RobotContainer robo = new RobotContainer();
    String pathName = "";

    public Auton() {}

    public Command RedRight() {
  pathName = "RedRightDynStarting";
    return new SequentialCommandGroup(
        robo.getShotCommand(),
        robo.intake(), 
        PathLoader.loadPath("RedRightDynStarting"),
        RedRightNext("zoneOne")
    );
}

public Command RedRightNext(String zone) {

    if (checkBoundingBox(zone)) {
      return new SequentialCommandGroup(
        robo.intake(),
        PathLoader.loadPath("RedRightDynNoteOne"),
        robo.getShotCommand()
      );
    }

    if (checkBoundingBox(zone)) {
      return new SequentialCommandGroup(
        robo.intake()
      );
    }

    if (checkBoundingBox(zone)) {
       return new SequentialCommandGroup(
        robo.intake()
      );     
    }

    return new SequentialCommandGroup();
  }

  public boolean checkBoundingBox(String zone) {

    return true;
  }

  public String getPathName() {

    return pathName;
  }
}
