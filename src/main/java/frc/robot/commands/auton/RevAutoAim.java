package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.RobotState.State;

public class RevAutoAim extends Command {
  private final RobotState robotState;

  public RevAutoAim(RobotState robotState) {
    this.robotState = robotState;
  }

  @Override
  public void initialize() {
    robotState.setState(State.AUTO_AIM_REVVING);
  }
}
