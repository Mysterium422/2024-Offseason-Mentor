package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.BeamBreak.BeamBreak;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Intake extends SubsystemBase {
  private RollerIO rollerIO;
  private LoggedDashboardNumber rollerSpeed = new LoggedDashboardNumber("Intake/Tuning/rollerSpeed");
  private RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();
  private BeamBreak beamBreak;

  public enum IntakeState {
    INTAKING,
    VOMITING,
    TUNING,
    IDLE,
    NOTE_HELD,
    SHOOTING
  }
  

  public IntakeState currentSetState = IntakeState.IDLE;

  public Intake(RollerIO rollerIO, BeamBreak beamBreak) {
    this.rollerIO = rollerIO;
    this.beamBreak = beamBreak;
  }

  public void setState(IntakeState to) {
    currentSetState = to;
  }

  @Override
  public void periodic() {

    Logger.recordOutput("Intake/State", currentSetState.toString());

    switch (currentSetState) {
      case INTAKING:
        rollerIO.setSpeed(1);
        if (beamBreak.isBroken()) {
          setState(IntakeState.NOTE_HELD);
        }
        break;
      case SHOOTING:
        rollerIO.setSpeed(1);
        break;
      case VOMITING:
        rollerIO.setSpeed(-1);
        break;
      case TUNING:
        rollerIO.setSpeed(rollerSpeed.get());
        break;
      case NOTE_HELD:
      case IDLE:
        rollerIO.setSpeed(0);
        break;
      default:
        break;
    }

    rollerIO.updateInputs(rollerIOInputs);
    Logger.processInputs("Intake/Roller", rollerIOInputs);
  }
}
