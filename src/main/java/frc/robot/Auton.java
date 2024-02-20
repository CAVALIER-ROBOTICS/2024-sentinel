package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonCommands.AutonPaths.BlueBottomCommand;
import frc.robot.commands.AutonCommands.AutonPaths.BlueMiddleCommand;
import frc.robot.commands.AutonCommands.AutonPaths.BlueTopCommand;
import frc.robot.commands.AutonCommands.AutonPaths.RedBottomCommand;
import frc.robot.commands.AutonCommands.AutonPaths.RedTopCommand;

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

    if (checkBoundaryBox("zoneOne") ) {
      return new SequentialCommandGroup(
        robo.intake(),
        PathLoader.loadPath("RedRightDynNoteOne"),
        robo.getShotCommand()
      );
    }

    if (checkBoundaryBox("zoneTwo")) {
      return new SequentialCommandGroup(
        robo.intake()
      );
    }

    if (checkBoundaryBox("zoneThree")) {
       return new SequentialCommandGroup(
        robo.intake()
      );     
    }

    return new SequentialCommandGroup();
  }

  public boolean checkBoundaryBox(String zone) {

    return true;
  }

  public String getPathName() {

    return pathName;
  }

    public Command getRedTop() {

    return new RedTopCommand();
  }

  public Command getRedMiddle() {

    return new RedBottomCommand();
  }

  public Command getRedBottom() {

    return new RedBottomCommand();
  }

  public Command getBlueTop() {

    return new BlueTopCommand();
  }

  public Command getBlueMiddle() {

    return new BlueMiddleCommand();
  }

  public Command getBlueBottom() {

    return new BlueBottomCommand();
  }
}
