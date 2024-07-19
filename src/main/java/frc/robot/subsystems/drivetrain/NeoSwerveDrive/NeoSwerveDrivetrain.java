// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain.NeoSwerveDrive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.NeoDrivetrainConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainInterface;
import frc.robot.Ports;
import frc.robot.RobotContainer;
import frc.robot.utils.NeoSwerveUtils;

/**
 * The {@code Drivetrain} class contains fields and methods pertaining to the function of the drivetrain.
 */
public class NeoSwerveDrivetrain extends SubsystemBase implements DrivetrainInterface {

 	// public static final double FRONT_LEFT_VIRTUAL_OFFSET_RADIANS = 1.340; // adjust as needed so that virtual (turn) position of wheel is zero when straight
	// public static final double FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS = -3.083; // adjust as needed so that virtual (turn) position of wheel is zero when straight
	// public static final double REAR_LEFT_VIRTUAL_OFFSET_RADIANS = -0.934; // adjust as needed so that virtual (turn) position of wheel is zero when straight
	// public static final double REAR_RIGHT_VIRTUAL_OFFSET_RADIANS = +1.021; // adjust as needed so that virtual (turn) position of wheel is zero when straight

	public static final double FRONT_LEFT_VIRTUAL_OFFSET_RADIANS = -3.03+Math.PI; // adjust as needed so that virtual (turn) position of wheel is zero when straight
	public static final double FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS = 2.69; // adjust as needed so that virtual (turn) position of wheel is zero when straight
	public static final double REAR_LEFT_VIRTUAL_OFFSET_RADIANS = -2.33; // adjust as needed so that virtual (turn) position of wheel is zero when straight
	public static final double REAR_RIGHT_VIRTUAL_OFFSET_RADIANS = Math.PI/2+Math.PI/16-Math.PI; // adjust as needed so that virtual (turn) position of wheel is zero when straight
    static final int GYRO_ORIENTATION = 1; // might be able to merge with kGyroReversed

	public static final double FIELD_LENGTH_INCHES = 54*12+1; // 54ft 1in
	public static final double FIELD_WIDTH_INCHES = 26*12+7; // 26ft 7in

	// turn settings
	// NOTE: it might make sense to decrease the PID controller period below 0.02 sec (which is the period used by the main loop)
	static final double TURN_PID_CONTROLLER_PERIOD_SECONDS = .02; // 0.01 sec = 10 ms 	
	
	static final double MIN_TURN_PCT_OUTPUT = 0.1; // 0.1;
	static final double MAX_TURN_PCT_OUTPUT = 0.4; // 0.4;
	
	static final double TURN_PROPORTIONAL_GAIN = 0.001; // 0.01;
	static final double TURN_INTEGRAL_GAIN = 0.0;
	static final double TURN_DERIVATIVE_GAIN = 0.0; // 0.0001
	
	static final int DEGREE_THRESHOLD = 10; // 3;
	
	private final static int TURN_ON_TARGET_MINIMUM_COUNT = 10; // number of times/iterations we need to be on target to really be on target
	// end turn settings	

	// Create SwerveModules
	private final NeoSwerveModule m_frontLeft /* #2 */ = new NeoSwerveModule(
		Ports.NeoDrive.FRONT_LEFT_DRIVING,
		Ports.NeoDrive.FRONT_LEFT_TURNING,
		Ports.NeoDrive.FRONT_LEFT_TURNING_ABSOLUTE_ENCODER,
		-0.317, false, "FrontLeft");

	private final NeoSwerveModule m_frontRight /* #1 */ = new NeoSwerveModule(
		Ports.NeoDrive.FRONT_RIGHT_DRIVING,
		Ports.NeoDrive.FRONT_RIGHT_TURNING,
		Ports.NeoDrive.FRONT_RIGHT_TURNING_ABSOLUTE_ENCODER,
		0.86, true, "FrontRight");

	private final NeoSwerveModule m_rearLeft /* #3 */ = new NeoSwerveModule(
		Ports.NeoDrive.REAR_LEFT_DRIVING,
		Ports.NeoDrive.REAR_LEFT_TURNING,
		Ports.NeoDrive.REAR_LEFT_TURNING_ABSOLUTE_ENCODER,
		-0.9, true, "RearLeft");

	private final NeoSwerveModule m_rearRight /* #4 */ = new NeoSwerveModule(
		Ports.NeoDrive.REAR_RIGHT_DRIVING,
		Ports.NeoDrive.REAR_RIGHT_TURNING,
		Ports.NeoDrive.REAR_RIGHT_TURNING_ABSOLUTE_ENCODER,
		-0.33, false, "RearRight");

	// The gyro sensor
	private final AHRS m_gyro = new AHRS(SPI.Port.kMXP); // usign SPI by default, which is what we want.

	public final WPI_Pigeon2 pigeon = new WPI_Pigeon2(5, "Sensors");

	// Slew rate filter variables for controlling lateral acceleration
	private double m_currentRotation = 0.0;
	private double m_currentTranslationDir = 0.0;
	private double m_currentTranslationMag = 0.0;

	private SlewRateLimiter m_magLimiter = new SlewRateLimiter(NeoDrivetrainConstants.MAGNITUDE_SLEW_RATE);
	private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(NeoDrivetrainConstants.ROTATIONAL_SLEW_RATE);
	private double m_prevTime = WPIUtilJNI.now() * 1e-6;

	// Odometry class for tracking robot pose
	SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
		NeoDrivetrainConstants.DRIVE_KINEMATICS,
		Rotation2d.fromDegrees(GYRO_ORIENTATION * pigeon.getYaw()),
		new SwerveModulePosition[] {
			m_frontLeft.getPosition(),
			m_frontRight.getPosition(),
			m_rearLeft.getPosition(),
			m_rearRight.getPosition()
		});


	// other variables
	private boolean isTurning;  // indicates that the drivetrain is turning using the PID controller hereunder

	private int onTargetCountTurning; // counter indicating how many times/iterations we were on target

	private PIDController turnPidController; // the PID controller used to turn

	private boolean speedBoost;

	/** Creates a new Drivetrain. */
	public NeoSwerveDrivetrain() {
		m_frontLeft.calibrateVirtualPosition(FRONT_LEFT_VIRTUAL_OFFSET_RADIANS); // set virtual position for absolute encoder
		m_frontRight.calibrateVirtualPosition(FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS);
		m_rearLeft.calibrateVirtualPosition(REAR_LEFT_VIRTUAL_OFFSET_RADIANS);
		m_rearRight.calibrateVirtualPosition(REAR_RIGHT_VIRTUAL_OFFSET_RADIANS);

		m_frontLeft.resetEncoders(); // resets relative encoders
		m_frontRight.resetEncoders();
		m_rearLeft.resetEncoders();
		m_rearRight.resetEncoders();

		zeroHeading(); // resets gyro

		// sets initial pose arbitrarily
		// Note: the field coordinate system (or global coordinate system) is an absolute coordinate system where a point on the field is designated as the origin.
		// Positive theta is in the counter-clockwise direction, and the positive x-axis points away from your alliance’s driver station wall,
		// and the positive y-axis is perpendicular and to the left of the positive x-axis.
		Translation2d initialTranslation = new Translation2d(Units.inchesToMeters(FIELD_LENGTH_INCHES/2),Units.inchesToMeters(FIELD_WIDTH_INCHES/2)); // mid field
		Rotation2d initialRotation = new Rotation2d(); 
		Pose2d initialPose = new Pose2d(initialTranslation,initialRotation);
		resetOdometry(initialPose);

		//creates a PID controller
		turnPidController = new PIDController(TURN_PROPORTIONAL_GAIN, TURN_INTEGRAL_GAIN, TURN_DERIVATIVE_GAIN);	
		
		turnPidController.enableContinuousInput(-180, 180); // because -180 degrees is the same as 180 degrees (needs input range to be defined first)
		turnPidController.setTolerance(DEGREE_THRESHOLD); // n degree error tolerated
	}

	public void resetHeading() {
		zeroHeading();
	}

	@Override
	public void configureDefaultCommand(CommandXboxController driverController) {
		setDefaultCommand(new RunCommand(
			() -> drive(
				RobotContainer.deadzone(driverController.getLeftY(), driverController.getLeftX(), driverController.getRightX(), Constants.JOYSTICK_THRESHOLD)*Constants.CONTROL_LIMITER,
				RobotContainer.deadzone(driverController.getLeftX(), driverController.getLeftY(), driverController.getRightX(), Constants.JOYSTICK_THRESHOLD)*Constants.CONTROL_LIMITER,
				RobotContainer.deadzone(driverController.getRightX(), driverController.getLeftY(), driverController.getLeftX(), Constants.JOYSTICK_THRESHOLD)*Constants.CONTROL_LIMITER,
		 	false), this));
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		m_odometry.update(
			Rotation2d.fromDegrees(GYRO_ORIENTATION * pigeon.getYaw()),
			new SwerveModulePosition[] {
				m_frontLeft.getPosition(),
				m_frontRight.getPosition(),
				m_rearLeft.getPosition(),
				m_rearRight.getPosition()
			});

		calculateTurnAngleUsingPidController();

		Logger.recordOutput("Robot/Pose", getPose());

		getFrontLeftModule().logMotorInfo();
		getFrontRightModule().logMotorInfo();
		getRearLeftModule().logMotorInfo();
		getRearRightModule().logMotorInfo();
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		m_odometry.resetPosition(
			Rotation2d.fromDegrees(GYRO_ORIENTATION * pigeon.getYaw()),
			new SwerveModulePosition[] {
				m_frontLeft.getPosition(),
				m_frontRight.getPosition(),
				m_rearLeft.getPosition(),
				m_rearRight.getPosition()
			},
			pose);
	}

	public void setBoost(boolean boost) {
		speedBoost = boost;
	}

	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		drive(xSpeed, ySpeed, rot, fieldRelative);
	}

	public void drive(double xSpeed, double ySpeed, double rot) {
		drive(xSpeed, ySpeed, rot, true);
	}

	public void drivePID(double xSpeed, double ySpeed, double rot) {
		SmartDashboard.putNumber("desired rotation", rot*Constants.NeoDrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND/(Math.PI*2));
		SmartDashboard.putNumber("pigeon rotation", pigeon.getRate() / 360);
		double error = 0;
		if (xSpeed != 0 || ySpeed != 0 || rot != 0) {
			error = rot*Constants.NeoDrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND/(Math.PI*2) - pigeon.getRate() / 360;
		}
		double kP = 1;
		drive(xSpeed, ySpeed, kP * error + rot);
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 * @param rateLimit     Whether to enable rate limiting for smoother control.
	 */
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
		
		double xSpeedCommanded;
		double ySpeedCommanded;

		if (rateLimit) {
			// Convert XY to polar for rate limiting
			double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
			double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

			// Calculate the direction slew rate based on an estimate of the lateral acceleration
			double directionSlewRate;

			if (m_currentTranslationMag != 0.0) {
				directionSlewRate = Math.abs(NeoDrivetrainConstants.DIRECTION_SLEW_RATE / m_currentTranslationMag);
			} else {
				directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
			}
			

			double currentTime = WPIUtilJNI.now() * 1e-6;
			double elapsedTime = currentTime - m_prevTime;
			double angleDif = NeoSwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);

			if (angleDif < 0.45*Math.PI) {
				m_currentTranslationDir = NeoSwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
				m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
			}
			else if (angleDif > 0.85*Math.PI) {
				if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
					// keep currentTranslationDir unchanged
					m_currentTranslationMag = m_magLimiter.calculate(0.0);
				}
				else {
					m_currentTranslationDir = NeoSwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
					m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
				}
			}
			else {
				m_currentTranslationDir = NeoSwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
				m_currentTranslationMag = m_magLimiter.calculate(0.0);
			}

			m_prevTime = currentTime;
			
			xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
			ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
			m_currentRotation = m_rotLimiter.calculate(rot);

		} else {
			xSpeedCommanded = xSpeed;
			ySpeedCommanded = ySpeed;
			m_currentRotation = rot;
		}

		// Convert the commanded speeds into the correct units for the drivetrain
		double maxSpeed;

		if (speedBoost) {
			maxSpeed = NeoDrivetrainConstants.MAX_SPEED_METERS_PER_SECOND_BOOSTED;
		} else {
			maxSpeed = NeoDrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
		}

		double xSpeedDelivered = xSpeedCommanded * maxSpeed;
		double ySpeedDelivered = ySpeedCommanded * maxSpeed;

		Logger.recordOutput("NeoSwerve/xSpeedDesired", xSpeedDelivered);
		Logger.recordOutput("NeoSwerve/ySpeedDesired", ySpeedDelivered);

		double rotDelivered = m_currentRotation * NeoDrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

		Logger.recordOutput("NeoSwerve/rotationDesired", rotDelivered);

		var swerveModuleStates = NeoDrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
			fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(GYRO_ORIENTATION * pigeon.getYaw()))
				: new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

		SwerveDriveKinematics.desaturateWheelSpeeds(
			swerveModuleStates, maxSpeed);

		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_rearLeft.setDesiredState(swerveModuleStates[2]);
		m_rearRight.setDesiredState(swerveModuleStates[3]);
	}

	/**
	 * Sets the wheels into an X formation to prevent movement.
	 */
	public void setX() {
		m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(
			desiredStates, NeoDrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);

		m_frontLeft.setDesiredState(desiredStates[0]);
		m_frontRight.setDesiredState(desiredStates[1]);
		m_rearLeft.setDesiredState(desiredStates[2]);
		m_rearRight.setDesiredState(desiredStates[3]);
	}

	/** Resets the drive encoders to currently read a position of 0 and seeds the turn encoders using the absolute encoders. */
	public void resetEncoders() {
		m_frontLeft.resetEncoders();
		m_rearLeft.resetEncoders();
		m_frontRight.resetEncoders();
		m_rearRight.resetEncoders();
	}

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		pigeon.setYaw(0);
	}

	public void oppositeHeading() {
		pigeon.setYaw(180);
	}

	public void stop()
	{
		drive(0, 0, 0, true, false);

		isTurning = false;
	}

	/** in dash
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		return Rotation2d.fromDegrees(-1 * GYRO_ORIENTATION * pigeon.getYaw()).getDegrees();
	}

	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second
	 */
	public double getTurnRate() {
		return m_gyro.getRate() * (NeoDrivetrainConstants.kGyroReversed ? -1.0 : 1.0);
	}

	public NeoSwerveModule getFrontLeftModule()
	{
		return m_frontLeft;
	}

	public NeoSwerveModule getFrontRightModule()
	{
		return m_frontRight;
	}

	public NeoSwerveModule getRearLeftModule()
	{
		return m_rearLeft;
	}

	public NeoSwerveModule getRearRightModule()
	{
		return m_rearRight;
	}

	public AHRS getImu()
	{
		return m_gyro;
	}

	public boolean isTurning(){
		return isTurning;
	}

	// this method needs to be paired with checkTurnAngleUsingPidController()
	public void turnAngleUsingPidController(double angle) {
		// switches to percentage vbus (vbux)
		
		stop(); // resets state
		
		double heading = getHeading() - angle;

		//System.out.println("requested heading " + heading);
		
		turnPidController.setSetpoint(heading); // sets the heading

		turnPidController.reset(); // resets controller
		
		isTurning = true;
		onTargetCountTurning = 0;
		//isReallyStalled = false;
		//stalledCount = 0;		
	}

	public void calculateTurnAngleUsingPidController() {	
		if (isTurning) {

			//System.out.println("current heading: " + getHeading());

			double output = MathUtil.clamp(turnPidController.calculate(getHeading()), -MAX_TURN_PCT_OUTPUT, MAX_TURN_PCT_OUTPUT);
			pidWriteRotation(output);
		}
	}

	// This method checks that we are within target up to ON_TARGET_MINIMUM_COUNT times
	// It relies on its own counter
	public boolean tripleCheckTurnAngleUsingPidController() {	
		if (isTurning) {
			boolean isOnTarget = turnPidController.atSetpoint();
			
			if (isOnTarget) { // if we are on target in this iteration 
				onTargetCountTurning++; // we increase the counter
			} else { // if we are not on target in this iteration
				if (onTargetCountTurning > 0) { // even though we were on target at least once during a previous iteration
					onTargetCountTurning = 0; // we reset the counter as we are not on target anymore
					System.out.println("Triple-check failed (turning).");
				} else {
					// we are definitely turning
				}
			}
			
			if (onTargetCountTurning > TURN_ON_TARGET_MINIMUM_COUNT) { // if we have met the minimum
				isTurning = false;
			}
			
			if (!isTurning) {
				System.out.println("You have reached the target (turning).");
				stop();				 
			}
		}
		return isTurning;
	}

	public void pidWriteRotation(double output) {

		//System.out.println("position error: " + turnPidController.getPositionError());
		//System.out.println("raw output: " + output);
		
		// calling disable() on controller will force a call to pidWrite with zero output
		// which we need to handle by not doing anything that could have a side effect 
		if (output != 0 && Math.abs(turnPidController.getPositionError()) < DEGREE_THRESHOLD)
		{
			output = 0;
		}
		if (output != 0 && Math.abs(output) < MIN_TURN_PCT_OUTPUT)
		{
			output = Math.signum(output) * MIN_TURN_PCT_OUTPUT;
		}

		//System.out.println("output: " + output);

		drive(0, 0, output, false); // TODO double-check sign
	}
}

//                          _______
//                         | -   - |
//                         | |   | |
//erik is birth today day! |_\ - /_|
//fortnite vbux              \___/