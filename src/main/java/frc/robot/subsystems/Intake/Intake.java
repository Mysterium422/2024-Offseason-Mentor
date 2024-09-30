package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIOInputsAutoLogged;

public class Intake extends SubsystemBase{

    private RollerIO m_rollerIO;
    private RollerIOInputsAutoLogged m_rollerIOInputs = new RollerIOInputsAutoLogged();

    public enum IntakeState {
        INTAKING,
        VOMITING,
        TUNING,
        IDLE,
        SHOOTING
    }

    private IntakeState currentState = IntakeState.IDLE;

    public Intake(RollerIO rollerIO) {
        this.m_rollerIO = rollerIO;
    }

    public void setState(IntakeState newState) {
        currentState = newState;
    }
 
    @Override
    public void periodic() {

        m_rollerIO.updateInputs(m_rollerIOInputs);
        Logger.processInputs("Intake/Roller",m_rollerIOInputs);

        Logger.recordOutput("Intake/State", currentState.toString());

        switch (currentState) {
            case INTAKING:
                m_rollerIO.setSpeed(1);
                break;
            case SHOOTING:
                m_rollerIO.setSpeed(1);
                break;
            case VOMITING:
                m_rollerIO.setSpeed(-1);
                break;
            case TUNING:
                // TODO
                break;
            case IDLE:
                m_rollerIO.setSpeed(0);
                break;
            default:
                break;
        }
    }
}
