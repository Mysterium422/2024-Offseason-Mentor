package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.Shooter.Feeder.FeederIO;
import frc.robot.subsystems.Shooter.Feeder.FeederIOInputsAutoLogged;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.Shooter.Pivot.PivotIO;
import frc.robot.subsystems.Shooter.Pivot.PivotIOInputsAutoLogged;
import frc.robot.utils.ShooterUtils;

public class Shooter extends SubsystemBase {

    private FlywheelIO m_flywheelIO;
    private PivotIO m_pivotIO;
    private FeederIO m_feederIO;
    private Limelight m_limelight;

    private FlywheelIOInputsAutoLogged m_flywheelIOInputs = new FlywheelIOInputsAutoLogged();
    private PivotIOInputsAutoLogged m_pivotIOInputs = new PivotIOInputsAutoLogged();
    private FeederIOInputsAutoLogged m_feederIOInputs = new FeederIOInputsAutoLogged();

    // For tuning the shooter. Only takes effect if in TUNING state.
    private LoggedDashboardNumber flywheelLeftSpeed = new LoggedDashboardNumber("Shooter/Tuning/FlywheelLeftSpeed");
    private LoggedDashboardNumber flywheelRightSpeed = new LoggedDashboardNumber("Shooter/Tuning/FlywheelRightSpeed");
    private LoggedDashboardNumber pivotAngle = new LoggedDashboardNumber("Shooter/Tuning/PivotAngle");
    private LoggedDashboardNumber rollerVoltage = new LoggedDashboardNumber("Shooter/Tuning/RollerVoltage");
    
    public enum ShooterState {
        AMP,
        AMP_REVVING,
        SUBWOOFER,
        SUBWOOFER_REVVING,
        AUTO_AIM,
        AUTO_AIM_REVVING,
        PASSING,
        VOMITING,
        TUNING,
        INTAKING,
        IDLE
    }

    private ShooterState currentState = ShooterState.IDLE;

    public Shooter(FlywheelIO flywheelIO, PivotIO pivotIO, FeederIO feederIO, Limelight limelight) {
        this.m_flywheelIO = flywheelIO;
        this.m_pivotIO = pivotIO;
        this.m_feederIO = feederIO;
        this.m_limelight = limelight;
    }

    public void setState(ShooterState to) {
        currentState = to;
    }

    public double getAngleFromDistance() {
        return ShooterUtils.instance.getAngleFromDistance(m_limelight.getDistance());
    }

    @Override
    public void periodic() {

        m_flywheelIO.updateInputs(m_flywheelIOInputs);
        Logger.processInputs("Shooter/Flywheel", m_flywheelIOInputs);

        m_pivotIO.updateInputs(m_pivotIOInputs);
        Logger.processInputs("Shooter/Pivot", m_pivotIOInputs);

        m_feederIO.updateInputs(m_feederIOInputs);
        Logger.processInputs("Shooter/Feeder", m_feederIOInputs);

        Logger.recordOutput("Shooter/State", currentState.toString());

        switch (currentState) {
            case AMP:
                m_flywheelIO.setSpeed(ShooterConstants.AMP_SPEED.get(), ShooterConstants.AMP_SPEED.get());
                m_pivotIO.setAngle(ShooterConstants.AMP_ANGLE.get());
                m_feederIO.setVoltage(ShooterConstants.AMP_ROLLER_VOLTAGE.get());
                break;
            case AMP_REVVING:
                m_flywheelIO.setSpeed(ShooterConstants.AMP_SPEED.get(), ShooterConstants.AMP_SPEED.get());
                m_pivotIO.setAngle(ShooterConstants.AMP_ANGLE.get());
                m_feederIO.setVoltage(0);
                break;
            case SUBWOOFER:
                m_flywheelIO.setSpeed(ShooterConstants.SPEAKER_SPEED.get() * 0.35, ShooterConstants.SPEAKER_SPEED.get());
                m_pivotIO.setAngle(ShooterConstants.SPEAKER_ANGLE.get());
                m_feederIO.setVoltage(ShooterConstants.SPEAKER_ROLLER_VOLTAGE.get());
                break;
            case SUBWOOFER_REVVING:
                m_flywheelIO.setSpeed(ShooterConstants.SPEAKER_SPEED.get() * 0.35, ShooterConstants.SPEAKER_SPEED.get());
                m_feederIO.setVoltage(0);
                m_pivotIO.setAngle(ShooterConstants.SPEAKER_ANGLE.get());
                break;
            case AUTO_AIM_REVVING:
                m_flywheelIO.setSpeed(ShooterConstants.SPEAKER_SPEED.get() * 0.35, ShooterConstants.SPEAKER_SPEED.get());
                m_feederIO.setVoltage(0);
                m_pivotIO.setAngle(getAngleFromDistance());
                break;
            case AUTO_AIM:
                m_flywheelIO.setSpeed(ShooterConstants.SPEAKER_SPEED.get() * 0.35, ShooterConstants.SPEAKER_SPEED.get());
                m_feederIO.setVoltage(ShooterConstants.SPEAKER_ROLLER_VOLTAGE.get());
                m_pivotIO.setAngle(getAngleFromDistance());
                break;
            case INTAKING:
                m_feederIO.setVoltage(ShooterConstants.INTAKING_ROLLER_VOLTAGE.get());
                break;
            case PASSING:
                // TODO
                break;
            case VOMITING:
                // TODO
                break;
            case TUNING:
                m_flywheelIO.setSpeed(flywheelLeftSpeed.get(), flywheelRightSpeed.get());
                m_pivotIO.setAngle(pivotAngle.get());
                m_feederIO.setVoltage(rollerVoltage.get());
                break;
            case IDLE:
                m_flywheelIO.setSpeed(0, 0);
                m_feederIO.setVoltage(0);
                break;
            default:
                break;
        }
        m_flywheelIO.updatePIDControllers();
    }
}
