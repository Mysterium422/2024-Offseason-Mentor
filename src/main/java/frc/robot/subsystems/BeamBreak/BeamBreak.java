package frc.robot.subsystems.BeamBreak;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotState.State;
import frc.robot.subsystems.BeamBreak.BeamBreakIO.BeamBreakIO;
import frc.robot.subsystems.BeamBreak.BeamBreakIO.BeamBreakIOInputsAutoLogged;

public class BeamBreak extends SubsystemBase{
    private RobotState m_robotState;

    private BeamBreakIO m_beamBreakIO;
    private BeamBreakIOInputsAutoLogged m_beamBreakIOInputs = new BeamBreakIOInputsAutoLogged();

    private XboxController m_driver;
    private XboxController m_operator;

    private int beamBreakBrokenTime;
    private BeamBreakState m_currentState = BeamBreakState.IDLE;

    public enum BeamBreakState {
        IDLE,
        NOTE_HELD,
        COUNTING
    }

    public BEAM_BREAK(BeamBreakIO beamBreakIO, RobotState robotState, int driverPort, int operatorPort) {
        this.m_beamBreakIO = beamBreakIO;
        this.m_robotState = robotState;
        this.m_driver = new XboxController(driverPort);
        this.m_operator = new XboxController(operatorPort);
    }

    @Override
    public void periodic() {

        if (beamBreakIO.getIsBroken()) {
            if (beamBreakBrokenTime == 0) {
                m_driver.setRumble(RumbleType.kBothRumble, 1);
                m_operator.setRumble(RumbleType.kBothRumble, 1);
            }
            if (beamBreakBrokenTime == IntakeConstants.BEAMBREAK_DELAY.get()) {
                m_robotState.currentState = State.NOTE_HELD;
            }
            if (beamBreakBrokenTime == IntakeConstants.CONTROLLER_RUMBLE_TIME.get()) {
                m_driver.setRumble(RumbleType.kBothRumble, 0);
                m_operator.setRumble(RumbleType.kBothRumble, 0);
            }
            beamBreakBrokenTime++;
        } else {
            beamBreakBrokenTime = 0;
        }

        Logger.recordOutput("BeamBreak/State", currentState.toString());
        Logger.recordOutput("BeamBreak/Counter", beamBreakBrokenTime);

        beamBreakIO.updateInputs(beamBreakIOInputs);
        Logger.processInputs("BeamBreak/Inputs", beamBreakIOInputs);
    }
}
