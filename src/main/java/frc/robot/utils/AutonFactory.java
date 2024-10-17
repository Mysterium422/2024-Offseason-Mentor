package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.RobotState.State;
import frc.robot.commands.auton.AlignWithSpeaker;
import frc.robot.commands.auton.ShootSubwoofer;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.Drivetrain.DriveState;
import frc.robot.subsystems.Limelight.Limelight;

import frc.robot.subsystems.Shooter.Shooter.ShooterState;

import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonFactory extends VirtualSubsystem {

  private final Drivetrain m_drivetrain;
  private final RobotState m_robotState;
  private final Limelight m_limelight;
  private final LoggedDashboardChooser<String> m_autonChooser;
  private String m_currAutonChoice;
  private Command m_currentAutonCommand;

  private LoggedTunableNumber PATHPLANNER_TRANSLATION_P =
      new LoggedTunableNumber("PathPlanner/Translation_P", 2.5);
  private LoggedTunableNumber PATHPLANNER_TRANSLATION_I =
      new LoggedTunableNumber("PathPlanner/Translation_I", 0);
  private LoggedTunableNumber PATHPLANNER_TRANSLATION_D =
      new LoggedTunableNumber("PathPlanner/Translation_D", 0);
  private LoggedTunableNumber PATHPLANNER_ROTATION_P =
      new LoggedTunableNumber("PathPlanner/ROTATION_P", 2);
  private LoggedTunableNumber PATHPLANNER_ROTATION_I =
      new LoggedTunableNumber("PathPlanner/ROTATION_I", 0);
  private LoggedTunableNumber PATHPLANNER_ROTATION_D =
      new LoggedTunableNumber("PathPlanner/ROTATION_D", 0);

  public AutonFactory(RobotState robotState, Drivetrain drivetrain, Limelight limelight) {
    this.m_drivetrain = drivetrain;
    this.m_robotState = robotState;
    this.m_limelight = limelight;

    m_autonChooser = new LoggedDashboardChooser<>("Auton Chooser");
    m_autonChooser.addOption("Do Nothing", "Do Nothing");
    m_autonChooser.addOption("Shoot Preload", "Shoot Preload");

    registerNamedCommands();

    List<String> paths = PathPlannerUtil.getExistingPaths();
    for (String path : paths) {
      m_autonChooser.addOption(path, path);
    }

    m_currAutonChoice = m_autonChooser.get();

    updateHolonomicConfig();

    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> Logger.recordOutput("PathPlanner/currentPose", pose));
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> Logger.recordOutput("PathPlanner/nextPose", pose));
  }

  private void updateHolonomicConfig() {
    HolonomicPathFollowerConfig pathFollowerConfig =
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(
                PATHPLANNER_TRANSLATION_P.get(),
                PATHPLANNER_TRANSLATION_I.get(),
                PATHPLANNER_TRANSLATION_D.get()), // Translation PID constants
            new PIDConstants(
                PATHPLANNER_ROTATION_P.get(),
                PATHPLANNER_ROTATION_I.get(),
                PATHPLANNER_ROTATION_D.get()), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.377, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            );

    AutoBuilder.configureHolonomic(
        m_drivetrain::getPose, // Robot pose supplier
        m_drivetrain
            ::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        m_drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        m_drivetrain
            ::setPathPlannerDesired, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        pathFollowerConfig,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        m_drivetrain // Reference to this subsystem to set requirements
        );

    m_currentAutonCommand = buildAutonCommand();
    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> Logger.recordOutput("PathPlanner/currentPose", pose));
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> Logger.recordOutput("PathPlanner/nextPose", pose));
  }

  private Command buildAutonCommand(String path) {
    /** Hard Coded Paths which will always work */
    if (path == null || path.equals("Do Nothing")) {
      return Commands.none();
    }
    if (path.equals("Shoot Preload")) {
      return new ShootSubwoofer(m_robotState);
    }

    System.out.println("Building Auto");
    Command autoCommand = AutoBuilder.buildAuto(path);
    return autoCommand;
  }

  private Command buildAutonCommand() {
    return buildAutonCommand(m_currAutonChoice);
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          updateHolonomicConfig();
        },
        PATHPLANNER_TRANSLATION_P,
        PATHPLANNER_TRANSLATION_I,
        PATHPLANNER_TRANSLATION_D,
        PATHPLANNER_ROTATION_P,
        PATHPLANNER_ROTATION_I,
        PATHPLANNER_ROTATION_D);

    String newAutonChoice = m_autonChooser.get();
    if (newAutonChoice == null && m_currAutonChoice == null) return;
    if (newAutonChoice == null || !newAutonChoice.equals(m_currAutonChoice)) {
      m_currAutonChoice = newAutonChoice;
      m_currentAutonCommand = buildAutonCommand();
    }
  }

  public Command getAutonCommand() {
    // System.out.println(m_currentAutonCommand == null);
    // System.out.println(m_currentAutonCommand.isFinished());
    // System.out.println(m_currentAutonCommand.isScheduled());
    if (m_currentAutonCommand == null || m_currentAutonCommand.isFinished()) {
      m_currentAutonCommand = buildAutonCommand();
    }

    return m_currentAutonCommand;
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand(
      "Idle",
      new InstantCommand(
        () -> {
          m_robotState.setState(State.IDLE);
        }));
  
    NamedCommands.registerCommand(
      "Rev",
      new InstantCommand(
        () -> {
          m_robotState.setState(State.SUBWOOFER_REVVING);
        }));
  
    NamedCommands.registerCommand(
      "SubwooferRev",
      new InstantCommand(() -> m_robotState.setState(State.SUBWOOFER_REVVING)));
  
    NamedCommands.registerCommand(
      "SubwooferShoot",
      new FunctionalCommand(
        () -> m_robotState.setState(State.SUBWOOFER_REVSHOOT),
        () -> {},
        (interrupted) -> {},
        () -> {
          return m_robotState.getShooterState() == ShooterState.SUBWOOFER;
        }
      ).andThen(new WaitCommand(0.5).andThen(new InstantCommand(() -> m_robotState.setState(State.INTAKE_REVVING)))));
  
    NamedCommands.registerCommand(
      "Intake",
      new InstantCommand(
        () -> {
          m_robotState.setState(State.INTAKE_REVVING);
        }));
  
    NamedCommands.registerCommand(
      "EnableVision",
      new InstantCommand(
        () -> {
          m_limelight.setAutonVisionEnabled(true);
        }));
  
    NamedCommands.registerCommand(
      "DisableVision",
      new InstantCommand(
        () -> {
          m_limelight.setAutonVisionEnabled(false);
        }));
  
    NamedCommands.registerCommand(
      "AutoAimRotate",
      new AlignWithSpeaker(m_limelight, m_drivetrain));
  
    NamedCommands.registerCommand(
      "AutoAimRevShoot",
      new FunctionalCommand(
        () -> m_robotState.setState(State.AUTO_AIM_REVSHOOT),
        () -> {},
        (interrupted) -> {},
        () -> {
          return m_robotState.getShooterState() == ShooterState.AUTO_AIM;
        }
      ).andThen(
        new WaitCommand(0.5).andThen(
          new InstantCommand(
            () -> m_robotState.setState(State.INTAKE_REVVING)
          )
        )
      ));
  }
}
