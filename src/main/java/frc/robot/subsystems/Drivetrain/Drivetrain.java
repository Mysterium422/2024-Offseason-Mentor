package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.SwerveDrivetrainIO.SwerveDrivetrainIO;
import frc.robot.subsystems.Drivetrain.SwerveDrivetrainIO.SwerveIOInputsAutoLogged;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.utils.ApriltagHelper.Tags;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {

  public enum DriveState {
    MANUAL,
    FOLLOW_PATH,
    SPEAKER_AUTO_ALIGN,
    X,
    TUNING
  }

  private final SwerveDrivetrainIO m_swerveDrivetrainIO;
  private final SwerveIOInputsAutoLogged swerveIOInputs = new SwerveIOInputsAutoLogged();
  private final Limelight m_limelight;

  private DriveState state = DriveState.MANUAL;

  private ChassisSpeeds driverChassisSpeeds; // Robot Relative
  // private double driveX;
  // private double driveY;
  // private double driveOmega;
  private ChassisSpeeds pathplannerChassisSpeeds; // Robot Relative

  private PIDController autoAimPID = new PIDController(0.02, 0, 0.001);
  private PIDController gyroAutoAimPID = new PIDController(0.035, 0, 0);

  private boolean speedBoost;

  public Drivetrain(SwerveDrivetrainIO swerveDrivetrainIO, Limelight limelight) {
    m_swerveDrivetrainIO = swerveDrivetrainIO;
    m_limelight = limelight;

    autoAimPID.setSetpoint(0);
    gyroAutoAimPID.enableContinuousInput(0, 360);
    if (Alliance.Red.equals(DriverStation.getAlliance().get())) {
      gyroAutoAimPID.setSetpoint(180);
    } else {
      gyroAutoAimPID.setSetpoint(0);
    }
  }

  @Override
  public void periodic() {
    m_swerveDrivetrainIO.updatePIDControllers();

    m_swerveDrivetrainIO.updateOdometry();
    m_swerveDrivetrainIO.updateOdometryWithVision(m_limelight);

    switch (state) {
      case SPEAKER_AUTO_ALIGN:
        double desiredOmega;
        if (m_limelight.isValidTarget(Tags.SPEAKER_CENTER.getId()) || m_limelight.isValidTarget(Tags.SPEAKER_OFFSET.getId())) {
          desiredOmega = autoAimPID.calculate(m_limelight.getTx());
        } else {
          desiredOmega = gyroAutoAimPID.calculate(m_swerveDrivetrainIO.getPigeonYaw());
        }

        m_swerveDrivetrainIO.drive(
          driverChassisSpeeds.vxMetersPerSecond,
          driverChassisSpeeds.vyMetersPerSecond,
          desiredOmega,
          false,
          false,
        speedBoost);

        break;
      case TUNING:
      case MANUAL:
        m_swerveDrivetrainIO.drive(driverChassisSpeeds, false, speedBoost);
        // m_swerveDrivetrainIO.drive(driveX, driveY, driveOmega, true, false, speedBoost);
        break;
      case FOLLOW_PATH:
        m_swerveDrivetrainIO.drive(pathplannerChassisSpeeds, false, true);
        break;
      case X:
        // m_swerveDrivetrainIO.drive(4,
        //     0,
        //     0,
        //     false, false, speedBoost
        // );
        m_swerveDrivetrainIO.drive(
            new SwerveModuleState[] {
              new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
              new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
              new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
              new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            });
        break;
    }

    // Logger.recordOutput("Drive/DrivetrainState", state.toString());

    // Original code has a calculateTurnAngleUsingPidController which seems to not do anything

    m_swerveDrivetrainIO.updateInputs(swerveIOInputs);
    swerveIOInputs.state = state;
    // swerveIOInputs.pose = getPose();
    Logger.processInputs("Drive", swerveIOInputs);
  }

  public void setState(DriveState state) {
    this.state = state;
  }

  public void setBoost(boolean boost) {
    speedBoost = boost;
  }

  // public void setDriverDesired(double driveX, double driveY, double driveOmega) {
  //     this.driveX = driveX;
  //     this.driveY = driveY;
  //     this.driveOmega = driveOmega;
  // }

  public void setDriverDesired(ChassisSpeeds speeds) {
    boolean discretizing = false;
    if ((Math.abs(speeds.vxMetersPerSecond) + Math.abs(speeds.vyMetersPerSecond)) > 1
        && Math.abs(speeds.omegaRadiansPerSecond) > 0.25) {
      speeds = ChassisSpeeds.discretize(speeds, 0.02);
      discretizing = true;
    }
    Logger.recordOutput("Drive/Discretizing", discretizing);
    this.driverChassisSpeeds = speeds;
  }

  public void setPathPlannerDesired(ChassisSpeeds speeds) {
    pathplannerChassisSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    Logger.recordOutput("Drive/PathPlannerSpeed", pathplannerChassisSpeeds);
  }

  public double getAngle() {
    return m_swerveDrivetrainIO.getPigeonYaw();
  }

  public void resetHeading() {
    m_swerveDrivetrainIO.resetHeading();
  }

  public Pose2d getPose() {
    return m_swerveDrivetrainIO.getPose();
  }

  public void resetOdometry(Pose2d pose) {
    m_swerveDrivetrainIO.resetOdometry(pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds output =
        Constants.NeoDrivetrainConstants.DRIVE_KINEMATICS.toChassisSpeeds(
            m_swerveDrivetrainIO.getSwerveModuleStates());
    return output;
  }
}
