package frc.robot.subsystems.SwerveDrivetrainNew;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.drivetrain.NeoSwerveDrive.NeoSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.NeoSwerveDrive.NeoSwerveModule;
import frc.robot.utils.AutonUtils;
import frc.robot.utils.LoggedTunableNumber;

public class SwerveDrivetrainNew extends SubsystemBase {
    private NeoSwerveDrivetrain drivetrain = new NeoSwerveDrivetrain();

    public enum SwerveDriveState {
        MANUAL,
        AUTO,
        SPEAKER_AUTO_ALIGN,
        X
    }   

    private Limelight limelight;

    private SwerveDriveState state = SwerveDriveState.MANUAL;

    private double driverChassisSpeedsX;
    private double driverChassisSpeedsY;
    private double driverChassisSpeedsRot;
    private double autoChassisSpeedsX;
    private double autoChassisSpeedsY;
    private double autoChassisSpeedsRot;

    private PIDController autoAimPID = new PIDController(0.02, 0, 0.001);

    public SwerveDrivetrainNew(Limelight limelight) {
        this.limelight = limelight;
        autoAimPID.setSetpoint(0);
    }

    @Override
    public void periodic() {
        switch (state) {
            case MANUAL:
                drivetrain.drive(
                    driverChassisSpeedsX,
                    driverChassisSpeedsY,
                    driverChassisSpeedsRot,
                    true
                );
                break;
            case AUTO:
                break;
            case SPEAKER_AUTO_ALIGN:
                if (limelight.isValidTarget(7)) {
                    drivetrain.drive(
                        driverChassisSpeedsX,
                        driverChassisSpeedsY,
                        -autoAimPID.calculate(limelight.getTx()),
                        true
                    );

                } else {
                    drivetrain.drive(
                        driverChassisSpeedsX,
                        driverChassisSpeedsY,
                        0,
                        true
                    );
                }
                break;
            case X:
                drivetrain.setX();
                break;
        }

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            autoAimPID.setP(Constants.AUTOAIM_P.get());
        });


        Logger.recordOutput("AutoAim/PIDOutput", autoAimPID.calculate(limelight.getTx()));
        Logger.recordOutput("AutoAim/PID_P", autoAimPID.getP());
        Logger.recordOutput("AutoAim/PID_D", autoAimPID.getD());
        Logger.recordOutput("currentSpeeds", getRobotRelativeSpeeds());
    }

    public void configureDefaultCommand(CommandXboxController driver) {
        drivetrain.setDefaultCommand(new RunCommand (() -> {
            driverChassisSpeedsX = RobotContainer.deadzone(driver.getLeftY(), driver.getLeftX(), driver.getRightX(), Constants.JOYSTICK_THRESHOLD)*Constants.CONTROL_LIMITER;
            driverChassisSpeedsY = RobotContainer.deadzone(driver.getLeftX(), driver.getLeftY(), driver.getRightX(), Constants.JOYSTICK_THRESHOLD)*Constants.CONTROL_LIMITER;
            driverChassisSpeedsRot = RobotContainer.deadzone(driver.getRightX(), driver.getLeftY(), driver.getLeftX(), Constants.JOYSTICK_THRESHOLD)*Constants.CONTROL_LIMITER;
        }, drivetrain));
    }

    public void setAutoDesiredSpeeds(double autoChassisSpeedsX, double autoChassisSpeedsY, double autoChassisSpeedsRot) {
        this.autoChassisSpeedsX = autoChassisSpeedsX;
        this.autoChassisSpeedsY = autoChassisSpeedsY;
        this.autoChassisSpeedsRot = autoChassisSpeedsRot;
    }

    public void setState(SwerveDriveState state) {
        this.state = state;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        ChassisSpeeds output = Constants.NeoDrivetrainConstants.DRIVE_KINEMATICS.toChassisSpeeds(
            drivetrain.getFrontLeftModule().getState(),
            drivetrain.getFrontRightModule().getState(),
            drivetrain.getRearLeftModule().getState(),
            drivetrain.getRearRightModule().getState()
        );
        return output;
    }

    public void resetHeading() {
        drivetrain.resetHeading();
    }

    public Pose2d getPose() {
        return drivetrain.getPose();
    }

    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
        drivetrain.drive(chassisSpeeds, fieldRelative);
    }

    // public ChassisSpeeds getRobotRelativeSpeeds() {
    //     return 
    // }

    public void resetOdometry(Pose2d pose) {
        drivetrain.resetOdometry(pose);
    }

    public void setBoost(boolean boost) {
        drivetrain.setBoost(boost);
    }
}
