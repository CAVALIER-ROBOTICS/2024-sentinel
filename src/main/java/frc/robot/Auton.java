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
        PathLoader.loadPath("RedTopDynStarting"),
        RedRightNext()
    );
}

public Command RedRightNext() {

    if (checkBoundingBox("zoneOne") ) {
      return new SequentialCommandGroup(
        robo.intake(),
        PathLoader.loadPath("RedRightDynNoteOne"),
        robo.getShotCommand()
      );
    }

    if (checkBoundingBox("zoneTwo")) {
      return new SequentialCommandGroup(
        robo.intake()
      );
    }

    if (checkBoundingBox("zoneThree")) {
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
