// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Ports.IntakePorts;
import frc.robot.subsystems.BeamBreak.BeamBreak;
import frc.robot.subsystems.BeamBreak.BeamBreakIO.BeamBreakIO;
import frc.robot.subsystems.BeamBreak.BeamBreakIO.BeamBreakIODIO;
import frc.robot.subsystems.BeamBreak.BeamBreakIO.BeamBreakIOSim;
import frc.robot.Ports.ShooterPorts;
import frc.robot.generated.TunerConstants;
import frc.robot.RobotState.State;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIO.ClimberIO;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIONeo;
import frc.robot.subsystems.Intake.Roller.RollerIOSim;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Feeder.FeederIO;
import frc.robot.subsystems.Shooter.Feeder.FeederIONeo;
import frc.robot.subsystems.Shooter.Feeder.FeederIOSim;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIONeo;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOSim;
import frc.robot.subsystems.Shooter.Pivot.PivotIO;
import frc.robot.subsystems.Shooter.Pivot.PivotIONeo;
import frc.robot.subsystems.Shooter.Pivot.PivotIOSim;
import frc.robot.subsystems.Climber.ClimberIO.ClimberIONeo;
import frc.robot.subsystems.Climber.ClimberIO.ClimberIOSim;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainInterface;
import frc.robot.subsystems.drivetrain.NeoSwerveDrive.NeoSwerveDrivetrain;

public class RobotContainer {

  private Climber climber;
  private Shooter shooter;
  private Intake intake;
  private BeamBreak beamBreak;

  private DrivetrainInterface drivetrain;

  private RobotState robotState;

  private final CommandXboxController driver = new CommandXboxController(Ports.driverControllerPort);
  private final CommandXboxController operator = new CommandXboxController(Ports.operatorControllerPort);

  public static double deadzone(double a, double b, double c, double zone) {
		if (Math.sqrt(Math.pow(a, 2)+Math.pow(b, 2)+Math.pow(c, 2)) > zone) {
			return a * Math.abs(a);
		} else {
			return 0;
		}
	}

  private void configureBindings() {

    drivetrain.configureDefaultCommand(driver);

    driver.a().onTrue(new InstantCommand(() -> drivetrain.resetHeading()));
    driver.x().onTrue(new InstantCommand(() -> drivetrain.setX()));

		operator.rightBumper().whileTrue(new StartEndCommand(() -> robotState.setState(State.INTAKING), () -> {
      if (robotState.currentState != State.NOTE_HELD) {
        robotState.setState(State.IDLE);
      }
    }, intake));

		operator.leftBumper().whileTrue(new StartEndCommand(() -> robotState.setState(State.VOMITING), () -> robotState.setState(State.IDLE), intake, shooter));
		operator.leftTrigger().whileTrue(new StartEndCommand(() -> robotState.setState(State.VOMITING), () -> robotState.setState(State.IDLE), intake, shooter));

		operator.rightTrigger().whileTrue(new StartEndCommand(() -> {
      switch (robotState.currentState) {
        case SUBWOOFER_REVVING:
          robotState.setState(State.SUBWOOFER);
          break;
        case AMP_REVVING:
          robotState.setState(State.AMP);
          break;
        default:
          break;
      }
    }, () -> robotState.setState(State.IDLE), shooter, intake));

		operator.a().whileTrue(new InstantCommand(() -> robotState.setState(State.SUBWOOFER_REVVING), shooter));
		operator.x().whileTrue(new InstantCommand(() -> robotState.setState(State.AMP_REVVING), shooter));
		operator.b().whileTrue(new InstantCommand(() -> robotState.setState(State.IDLE), shooter, intake));
    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    // drivetrain.registerTelemetry(logger::telemeterize);

  }

  public void initializeSubsystems() {

    // Drivetrain

    if (Constants.USE_KRAKEN_DRIVETRAIN.get()) { // default value is false which means neo is used
      drivetrain = TunerConstants.DriveTrain;
    } else {
      drivetrain = new NeoSwerveDrivetrain();
    }

    // Shooter

    FlywheelIO flywheelIO;
    PivotIO pivotIO;
    FeederIO feederIO;

    if (Constants.currentMode == Constants.Mode.REAL) {
      flywheelIO = new FlywheelIONeo(ShooterPorts.leftFlywheel, ShooterPorts.rightFlywheel);
      pivotIO = new PivotIONeo(ShooterPorts.pivot, ShooterPorts.pivotEncoder);
      feederIO = new FeederIONeo(ShooterPorts.rollerTop, ShooterPorts.rollerBottom);
    } else {
      flywheelIO = new FlywheelIOSim();
      pivotIO = new PivotIOSim();
      feederIO = new FeederIOSim();
    }

    shooter = new Shooter(flywheelIO, pivotIO, feederIO);

    // Intake

    RollerIO rollerIO;

    if (Constants.currentMode == Constants.Mode.REAL) {
      rollerIO = new RollerIONeo(IntakePorts.runExternal, IntakePorts.runInternal);
    } else {
      rollerIO = new RollerIOSim();
    }

    intake = new Intake(rollerIO);

    // Climber

    ClimberIO climberIO;

    if (Constants.currentMode == Constants.Mode.REAL){
      climberIO = new ClimberIONeo(Ports.ClimberPorts.leftClimberID, Ports.ClimberPorts.rightClimberID);
    } else {
      climberIO = new ClimberIOSim();
    }

    climber = new Climber(operator, climberIO);
    
        // Robot State

    robotState = new RobotState(shooter, climber, intake);

    // BeamBreak

    BeamBreakIO beamBreakIO;

    if (Constants.currentMode == Constants.Mode.REAL) {
      beamBreakIO = new BeamBreakIODIO(IntakePorts.beamBreak);
    } else {
      beamBreakIO = new BeamBreakIOSim();
    }

    beamBreak = new BeamBreak(beamBreakIO, robotState, Ports.driverControllerPort, Ports.operatorControllerPort);

  }

  public RobotContainer() { 
    initializeSubsystems();
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
