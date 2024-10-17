package frc.robot.subsystems.Drivetrain.SwerveDrivetrainIO;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.Constants.NeoDrivetrainConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Ports;
import frc.robot.subsystems.Drivetrain.SwerveModuleIO.SwerveModuleIOInputsAutoLogged;
import frc.robot.subsystems.Drivetrain.SwerveModuleIO.SwerveModuleIONeo;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.NeoSwerveUtils;
import org.littletonrobotics.junction.Logger;

public class SwerveDrivetrainIONeo implements SwerveDrivetrainIO {

  private static final LoggedTunableNumber FRONT_LEFT_VIRTUAL_OFFSET_RADIANS =
      new LoggedTunableNumber(
          "Drive/Tuning/FrontLeftOffset",
          0.125
              + Math.PI); // adjust as needed so that virtual (turn) position of wheel is zero when
  // straight
  private static final LoggedTunableNumber FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS =
      new LoggedTunableNumber(
          "Drive/Tuning/FrontRightOffset",
          2.657
              + Math.PI); // adjust as needed so that virtual (turn) position of wheel is zero when
  // straight
  private static final LoggedTunableNumber REAR_LEFT_VIRTUAL_OFFSET_RADIANS =
      new LoggedTunableNumber(
          "Drive/Tuning/RearLeftOffset",
          0.8); // adjust as needed so that virtual (turn) position of wheel is zero when straight
  private static final LoggedTunableNumber REAR_RIGHT_VIRTUAL_OFFSET_RADIANS =
      new LoggedTunableNumber("Drive/Tuning/RearRightOffset", 1.78);
  private static final int GYRO_ORIENTATION = 1; // might be able to merge with kGyroReversed

  private static final double FIELD_LENGTH_INCHES = 54 * 12 + 1; // 54ft 1in
  private static final double FIELD_WIDTH_INCHES = 26 * 12 + 7; // 26ft 7in

  /** TURN SETTINGS */
  // NOTE: it might make sense to decrease the PID controller period below 0.02 sec (which is the
  // period used by the main loop)
  private static final double TURN_PID_CONTROLLER_PERIOD_SECONDS = .02; // 0.01 sec = 10 ms

  private static final double MIN_TURN_PCT_OUTPUT = 0.1; // 0.1;
  private static final double MAX_TURN_PCT_OUTPUT = 0.4; // 0.4;

  private static final double TURN_PROPORTIONAL_GAIN = 0.001; // 0.01;
  private static final double TURN_INTEGRAL_GAIN = 0.0;
  private static final double TURN_DERIVATIVE_GAIN = 0.0; // 0.0001

  private static final int DEGREE_THRESHOLD = 10; // 3;

  private static final int TURN_ON_TARGET_MINIMUM_COUNT =
      10; // number of times/iterations we need to be on target to really be on target

  /** END TURN SETTINGS */

  // Create SwerveModules
  private final SwerveModuleIONeo m_frontLeft /* #2 */ =
      new SwerveModuleIONeo(
          Ports.NeoDrive.FRONT_LEFT_DRIVING,
          Ports.NeoDrive.FRONT_LEFT_TURNING,
          Ports.NeoDrive.FRONT_LEFT_TURNING_ABSOLUTE_ENCODER,
          FRONT_LEFT_VIRTUAL_OFFSET_RADIANS.get(),
          false,
          "FrontLeft");

  private final SwerveModuleIONeo m_frontRight /* #1 */ =
      new SwerveModuleIONeo(
          Ports.NeoDrive.FRONT_RIGHT_DRIVING,
          Ports.NeoDrive.FRONT_RIGHT_TURNING,
          Ports.NeoDrive.FRONT_RIGHT_TURNING_ABSOLUTE_ENCODER,
          FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS.get(),
          false,
          "FrontRight");

  private final SwerveModuleIONeo m_rearLeft /* #3 */ =
      new SwerveModuleIONeo(
          Ports.NeoDrive.REAR_LEFT_DRIVING,
          Ports.NeoDrive.REAR_LEFT_TURNING,
          Ports.NeoDrive.REAR_LEFT_TURNING_ABSOLUTE_ENCODER,
          REAR_LEFT_VIRTUAL_OFFSET_RADIANS.get(),
          false,
          "RearLeft");

  private final SwerveModuleIONeo m_rearRight /* #4 */ =
      new SwerveModuleIONeo(
          Ports.NeoDrive.REAR_RIGHT_DRIVING,
          Ports.NeoDrive.REAR_RIGHT_TURNING,
          Ports.NeoDrive.REAR_RIGHT_TURNING_ABSOLUTE_ENCODER,
          REAR_RIGHT_VIRTUAL_OFFSET_RADIANS.get(),
          false,
          "RearRight");

  private final SwerveModuleIOInputsAutoLogged m_frontLeftInputs =
      new SwerveModuleIOInputsAutoLogged();
  private final SwerveModuleIOInputsAutoLogged m_frontRightInputs =
      new SwerveModuleIOInputsAutoLogged();
  private final SwerveModuleIOInputsAutoLogged m_rearLeftInputs =
      new SwerveModuleIOInputsAutoLogged();
  private final SwerveModuleIOInputsAutoLogged m_rearRightInputs =
      new SwerveModuleIOInputsAutoLogged();

  private final WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(5, "Sensors");

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter =
      new SlewRateLimiter(NeoDrivetrainConstants.MAGNITUDE_SLEW_RATE);
  private SlewRateLimiter m_rotLimiter =
      new SlewRateLimiter(NeoDrivetrainConstants.ROTATIONAL_SLEW_RATE);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // other variables
  private boolean
      isTurning; // indicates that the drivetrain is turning using the PID controller hereunder

  private int
      onTargetCountTurning; // counter indicating how many times/iterations we were on target

  private PIDController m_turnPidController; // the PID controller used to turn

  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          NeoDrivetrainConstants.DRIVE_KINEMATICS,
          Rotation2d.fromDegrees(getPigeonYaw()),
          getSwerveModulePositions(),
          new Pose2d());

  public SwerveDrivetrainIONeo() {
    m_turnPidController =
        new PIDController(TURN_PROPORTIONAL_GAIN, TURN_INTEGRAL_GAIN, TURN_DERIVATIVE_GAIN);
    m_turnPidController.enableContinuousInput(-180, 180);
    m_turnPidController.setTolerance(DEGREE_THRESHOLD);

    m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));

    m_frontLeft.calibrateVirtualPosition(FRONT_LEFT_VIRTUAL_OFFSET_RADIANS.get());
    m_frontRight.calibrateVirtualPosition(FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS.get());
    m_rearLeft.calibrateVirtualPosition(REAR_LEFT_VIRTUAL_OFFSET_RADIANS.get());
    m_rearRight.calibrateVirtualPosition(REAR_RIGHT_VIRTUAL_OFFSET_RADIANS.get());
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_frontLeft.calibrateVirtualPosition(FRONT_LEFT_VIRTUAL_OFFSET_RADIANS.get());
        },
        FRONT_LEFT_VIRTUAL_OFFSET_RADIANS);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_frontRight.calibrateVirtualPosition(FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS.get());
        },
        FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_rearLeft.calibrateVirtualPosition(REAR_LEFT_VIRTUAL_OFFSET_RADIANS.get());
        },
        REAR_LEFT_VIRTUAL_OFFSET_RADIANS);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_rearRight.calibrateVirtualPosition(REAR_RIGHT_VIRTUAL_OFFSET_RADIANS.get());
        },
        REAR_RIGHT_VIRTUAL_OFFSET_RADIANS);

    m_frontLeft.updateInputs(m_frontLeftInputs);
    m_frontRight.updateInputs(m_frontRightInputs);
    m_rearLeft.updateInputs(m_rearLeftInputs);
    m_rearRight.updateInputs(m_rearRightInputs);
    Logger.processInputs("Drive/FrontLeft", m_frontLeftInputs);
    Logger.processInputs("Drive/FrontRight", m_frontRightInputs);
    Logger.processInputs("Drive/RearLeft", m_rearLeftInputs);
    Logger.processInputs("Drive/RearRight", m_rearRightInputs);
    inputs.pose = getPose();
    inputs.currentStates = getSwerveModuleStates();
    inputs.desiredStates = getSwerveModuleDesiredStates();
    inputs.pigeonYaw = getPigeonYaw();
  }

  @Override
  public void resetHeading() {
    m_pigeon.setYaw(0);
  }

  @Override
  public void drive(
      double xSpeed,
      double ySpeed,
      double rot,
      boolean fieldRelative,
      boolean rateLimit,
      boolean speedBoost) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;

      if (m_currentTranslationMag != 0.0) {
        directionSlewRate =
            Math.abs(NeoDrivetrainConstants.DIRECTION_SLEW_RATE / m_currentTranslationMag);
      } else {
        directionSlewRate =
            500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif =
          NeoSwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);

      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir =
            NeoSwerveUtils.StepTowardsCircular(
                m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag
            > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = NeoSwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir =
            NeoSwerveUtils.StepTowardsCircular(
                m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
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

    double rotDelivered =
        m_currentRotation * NeoDrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

    SwerveModuleState[] swerveModuleStates =
        NeoDrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(getPigeonYaw()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  @Override
  public double getPigeonYaw() {
    return m_pigeon.getYaw() * GYRO_ORIENTATION;
  }

  @Override
  public SwerveModuleIONeo getFrontLeftModule() {
    return m_frontLeft;
  }

  @Override
  public SwerveModuleIONeo getFrontRightModule() {
    return m_frontRight;
  }

  @Override
  public SwerveModuleIONeo getRearLeftModule() {
    return m_rearLeft;
  }

  @Override
  public SwerveModuleIONeo getRearRightModule() {
    return m_rearRight;
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    m_pigeon.setYaw(pose.getRotation().getDegrees());
    m_poseEstimator.resetPosition(pose.getRotation(), getSwerveModulePositions(), pose);
  }

  public void updateOdometry() {
    m_poseEstimator.update(Rotation2d.fromDegrees(0), getSwerveModulePositions());
  }

  public void updateOdometryWithVision(Limelight limelight) {
    LimelightHelpers.PoseEstimate mt2 = limelight.getMegatagPose(getPose());

    if (mt2 != null) {
      m_poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
      Logger.recordOutput("Limelight/MegatagPose", mt2.pose);
    }
  }

  public void updatePIDControllers() {
    m_frontLeft.updatePIDControllers();
    m_frontRight.updatePIDControllers();
    m_rearLeft.updatePIDControllers();
    m_rearRight.updatePIDControllers();
  }
}
