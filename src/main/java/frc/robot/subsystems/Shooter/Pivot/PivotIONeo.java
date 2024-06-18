package frc.robot.subsystems.Shooter.Pivot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.ShooterConstants;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class PivotIONeo implements PivotIO {
    private CANSparkMax pivotNeo;
    private PIDController pivotPID;
    private DutyCycleEncoder pivotEncoder;

    public PivotIONeo(int pivotNeoID, int pivotEncoderDIOID) {
        pivotNeo = new CANSparkMax(pivotNeoID, MotorType.kBrushless);
        pivotNeo.setIdleMode(IdleMode.kCoast);
        pivotNeo.burnFlash();

        pivotEncoder = new DutyCycleEncoder(new DigitalInput(pivotEncoderDIOID));

        pivotPID = new PIDController(ShooterConstants.PIVOT_P.get(), ShooterConstants.PIVOT_I.get(), ShooterConstants.PIVOT_D.get());
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        
    }

    @Override
    public void setAngle(double angle) {

        double encoderReading = pivotEncoder.getAbsolutePosition();

        if (encoderReading < 0.5) {
            encoderReading++;
        }

        Logger.recordOutput("Shooter/DesiredPivotAngle", angle);
        Logger.recordOutput("Shooter/PivotAngle", encoderReading);

        double pidAmount = pivotPID.calculate(encoderReading, Math.min(Math.max(angle, ShooterConstants.MIN_PIVOT_ANGLE.get()), ShooterConstants.MAX_PIVOT_ANGLE.get()));

        pidAmount*= 12; // multiply by 12 because the battery is 12 volts
        
        Logger.recordOutput("Shooter/PowerAppliedToPivot", pidAmount);

        pivotNeo.setVoltage(pidAmount);
    }
}