package frc.robot.sensors.BeamBreak;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.littletonrobotics.VirtualSubsystem;
import frc.robot.sensors.BeamBreak.BeamBreakIO.BeamBreakIO;
import frc.robot.sensors.BeamBreak.BeamBreakIO.BeamBreakIOInputsAutoLogged;

public class BeamBreak extends VirtualSubsystem {
  private BeamBreakIO beamBreakIO;
  private BeamBreakIOInputsAutoLogged inputs = new BeamBreakIOInputsAutoLogged();
  private double initialBreakTimestamp;
  private boolean brokenLastCycle;

  public BeamBreak(BeamBreakIO beamBreakIO) {
    this.beamBreakIO = beamBreakIO;
  }

  public boolean isBroken() {
    return inputs.isBroken && brokenLastCycle;
  }

  public double getBrokenTime() {
    if (isBroken()) {
      return Timer.getFPGATimestamp() - initialBreakTimestamp;
    } else {
      return 0.0;
    }
  }

  @Override
  public void periodic() {
    beamBreakIO.updateInputs(inputs);

    if (inputs.isBroken && !brokenLastCycle) {
      initialBreakTimestamp = Timer.getFPGATimestamp();
      brokenLastCycle = true;
    } else if (!beamBreakIO.getIsBroken()) {
      brokenLastCycle = false;
    }
    
    Logger.recordOutput("BrokenTwice", isBroken());
    Logger.recordOutput("BrokenTime", getBrokenTime());
  }
}
