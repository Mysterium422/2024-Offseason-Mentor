package frc.robot.sensors.BeamBreak.BeamBreakIO;

public class BeamBreakIOSim implements BeamBreakIO {
  @Override
  public boolean getIsBroken() {
    return false;
  }

  @Override
  public void updateInputs(BeamBreakIOInputs inputs) {
    inputs.isBroken = getIsBroken();
  }
}
