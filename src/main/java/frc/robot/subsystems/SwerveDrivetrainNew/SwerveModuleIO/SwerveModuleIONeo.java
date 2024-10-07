package frc.robot.subsystems.SwerveDrivetrainNew.SwerveModuleIO;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.NeoSwerveModuleConstants;
import frc.robot.utils.LoggedTunableNumber;

public class SwerveModuleIONeo implements SwerveModuleIO {
    private final CANSparkMax m_drivingSparkMax;
	private final CANSparkMax m_turningSparkMax;

	private final RelativeEncoder m_drivingEncoder;
	private final RelativeEncoder m_turningEncoder;
	private final CANCoder m_turningAbsoluteEncoder;

	private final SparkMaxPIDController m_drivingPIDController;
	private final SparkMaxPIDController m_turningPIDController;
	private double offset = 0;

    private final String moduleName;

    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModuleIONeo(int drivingCANId, int turningCANId, int turningAnalogPort, double offset, boolean inverted, String moduleName) {
		this.moduleName = moduleName;

		m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
		m_drivingSparkMax.setInverted(inverted);
		m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);
		this.offset = offset;

		// Factory reset, so we get the SPARKS MAX to a known state before configuring
		// them. This is useful in case a SPARK MAX is swapped out.
		m_drivingSparkMax.restoreFactoryDefaults();
		m_turningSparkMax.restoreFactoryDefaults();

		// Setup encoders and PID controllers for the driving and turning SPARKS MAX.
		m_drivingEncoder = m_drivingSparkMax.getEncoder();
		m_turningEncoder = m_turningSparkMax.getEncoder();
		m_turningAbsoluteEncoder = new CANCoder(turningAnalogPort, "Sensors");
		CANCoderConfiguration config = new CANCoderConfiguration();
		config.sensorCoefficient = 2*Math.PI/4096;
		config.unitString = "rad";
		config.sensorTimeBase = SensorTimeBase.PerSecond;
		config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
 		m_turningAbsoluteEncoder.configAllSettings(config);
		m_turningAbsoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100);

		m_drivingPIDController = m_drivingSparkMax.getPIDController();
		m_turningPIDController = m_turningSparkMax.getPIDController();
		m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
		m_turningPIDController.setFeedbackDevice(m_turningEncoder);

		// Apply position and velocity conversion factors for the driving encoder. The
		// native units for position and velocity are rotations and RPM, respectively,
		// but we want meters and meters per second to use with WPILib's swerve APIs.
		m_drivingEncoder.setPositionConversionFactor(NeoSwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION);
		m_drivingEncoder.setVelocityConversionFactor(NeoSwerveModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_RPM);

		// Apply position and velocity conversion factors for the turning encoder. We
		// want these in radians and radians per second to use with WPILib's swerve APIs.
		m_turningEncoder.setPositionConversionFactor(NeoSwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION);
		m_turningEncoder.setVelocityConversionFactor(NeoSwerveModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM);

		// Invert the turning controller, since the output shaft rotates in the opposite direction of
		// the steering motor.
		m_turningSparkMax.setInverted(true);


		// Enable PID wrap around for the turning motor. This will allow the PID
		// controller to go through 0 to get to the setpoint i.e. going from 350 degrees
		// to 10 degrees will go through 0 rather than the other direction which is a
		// longer route.
		m_turningPIDController.setPositionPIDWrappingEnabled(true);
		m_turningPIDController.setPositionPIDWrappingMinInput(NeoSwerveModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS);
		m_turningPIDController.setPositionPIDWrappingMaxInput(NeoSwerveModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS);

		// Set the PID gains for the driving motor.
		m_drivingPIDController.setP(NeoSwerveModuleConstants.DRIVING_P.get());
		m_drivingPIDController.setI(NeoSwerveModuleConstants.DRIVING_I.get());
		m_drivingPIDController.setD(NeoSwerveModuleConstants.DRIVING_D.get());
		m_drivingPIDController.setFF(NeoSwerveModuleConstants.DRIVING_FF.get());
		m_drivingPIDController.setOutputRange(NeoSwerveModuleConstants.DRIVING_MIN_OUTPUT_NORMALIZED, NeoSwerveModuleConstants.DRIVING_MAX_OUTPUT_NORMALIZED);

		// Set the PID gains for the turning motor.
		m_turningPIDController.setP(NeoSwerveModuleConstants.TURNING_P.get());
		m_turningPIDController.setI(NeoSwerveModuleConstants.TURNING_I.get());
		m_turningPIDController.setD(NeoSwerveModuleConstants.TURNING_D.get());
		m_turningPIDController.setFF(NeoSwerveModuleConstants.TURNING_FF.get());
		m_turningPIDController.setOutputRange(NeoSwerveModuleConstants.TURNING_MIN_OUTPUT_NORMALIZED, NeoSwerveModuleConstants.TURNING_MAX_OUTPUT_NORMALIZED);

		m_drivingSparkMax.setIdleMode(NeoSwerveModuleConstants.DRIVING_MOTOR_IDLE_MODE);
		m_turningSparkMax.setIdleMode(NeoSwerveModuleConstants.TURNING_MOTOR_IDLE_MODE);
		m_drivingSparkMax.setSmartCurrentLimit(NeoSwerveModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT_AMPS);
		m_turningSparkMax.setSmartCurrentLimit(NeoSwerveModuleConstants.TURNING_MOTOR_CURRENT_LIMIT_AMPS);

		// Save the SPARK MAX configurations. If a SPARK MAX browns out during
		// operation, it will maintain the above configurations.
		m_drivingSparkMax.burnFlash();
		m_turningSparkMax.burnFlash();

		m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
		m_drivingEncoder.setPosition(0);
	}

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.turningCurrentAmps = m_turningSparkMax.getOutputCurrent();
        inputs.turningTempFahrenheit = m_turningSparkMax.getMotorTemperature();
        inputs.turningVelocityRPM = m_turningSparkMax.getEncoder().getVelocity();
        inputs.turningIsOn = Math.abs(m_turningSparkMax.getAppliedOutput()) > 0.01;
        inputs.turningVoltage = m_turningSparkMax.getAppliedOutput() * m_turningSparkMax.getBusVoltage();

        inputs.drivingCurrentAmps = m_drivingSparkMax.getOutputCurrent();
        inputs.drivingTempFahrenheit = m_drivingSparkMax.getMotorTemperature();
        inputs.drivingVelocityRPM = m_drivingSparkMax.getEncoder().getVelocity();
        inputs.drivingIsOn = Math.abs(m_drivingSparkMax.getAppliedOutput()) > 0.01;
        inputs.drivingVoltage = m_drivingSparkMax.getAppliedOutput() * m_drivingSparkMax.getBusVoltage();

        inputs.currentState = getState();
        inputs.desiredState = getDesiredState();
		inputs.offset = offset;
    }

    @Override
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
        m_turningSparkMax.set(0);
        m_turningEncoder.setPosition(m_turningAbsoluteEncoder.getAbsolutePosition() + offset);
    }

    @Override
    public void calibrateVirtualPosition(double angle) {
		if (this.offset != angle) {
			m_turningEncoder.setPosition(m_turningAbsoluteEncoder.getAbsolutePosition() + angle);
		}
        this.offset = angle;
    }

    @Override
    public double getOffset() {
        return offset;
    }

    @Override
    public RelativeEncoder getDrivingEncoder() {
        return m_drivingEncoder;
    }

    @Override
    public RelativeEncoder getTurningEncoder() {
        return m_turningEncoder;
    }

    @Override
    public CANCoder getTurningAbsoluteEncoder() {
        return m_turningAbsoluteEncoder;
    }

    @Override
    public SwerveModuleState getDesiredState() {
        return m_desiredState;
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getPosition()));

        // NOTE: Changed from working swerve code. Original just set this to desired state
        m_desiredState = optimizedState;

        if (Math.abs(optimizedState.speedMetersPerSecond) < 0.001 && Math.abs(optimizedState.angle.getRadians() - m_turningEncoder.getPosition()) < Rotation2d.fromDegrees(1).getRadians()) {
            m_drivingSparkMax.set(0);
            m_turningSparkMax.set(0);
            return;
        }

        m_drivingPIDController.setReference(optimizedState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        m_turningPIDController.setReference(optimizedState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_drivingEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_drivingEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition()));
    }

    @Override
    public void updatePIDControllers() {
		LoggedTunableNumber.ifChanged(hashCode(), () -> {
			m_drivingPIDController.setP(Constants.NeoSwerveModuleConstants.DRIVING_P.get());
			m_drivingPIDController.setI(Constants.NeoSwerveModuleConstants.DRIVING_I.get());
			m_drivingPIDController.setD(Constants.NeoSwerveModuleConstants.DRIVING_D.get());
			m_drivingPIDController.setFF(Constants.NeoSwerveModuleConstants.DRIVING_FF.get());
		});

		LoggedTunableNumber.ifChanged(hashCode(), () -> {
			m_turningPIDController.setP(Constants.NeoSwerveModuleConstants.TURNING_P.get());
			m_turningPIDController.setI(Constants.NeoSwerveModuleConstants.TURNING_I.get());
			m_turningPIDController.setD(Constants.NeoSwerveModuleConstants.TURNING_D.get());
			m_turningPIDController.setFF(Constants.NeoSwerveModuleConstants.TURNING_FF.get());
		});
    }
    
}
