package frc.robot.sensors.BeamBreak.BeamBreakIO;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakIODIO implements BeamBreakIO {
  private DigitalInput beamBreakDigitalInput;

  public BeamBreakIODIO(int beamBreakDIOID) {
    beamBreakDigitalInput = new DigitalInput(beamBreakDIOID);
  }

  @Override
  public boolean getIsBroken() {
    return beamBreakDigitalInput.get();
  }

  public void updateInputs(BeamBreakIOInputs inputs) {
    inputs.isBroken = getIsBroken();
  }
}
