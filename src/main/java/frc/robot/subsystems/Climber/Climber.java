package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberIO.ClimberIO;
import frc.robot.subsystems.Climber.ClimberIO.ClimberIOInputsAutoLogged;;

public class Climber extends SubsystemBase{
    private CommandXboxController operator;    
    private ClimberIO climberIO;
    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(CommandXboxController operator, ClimberIO climberIO) {
        this.operator = operator;
        this.climberIO = climberIO;
    }

    @Override
    public void periodic() {        
        climberIO.setPower(Constants.deadzone(-operator.getRightY()), Constants.deadzone(-operator.getLeftY()));
        climberIO.updateInputs(inputs);
        Logger.recordOutput("leftOpY", operator.getLeftY());
        Logger.recordOutput("rightOpY", operator.getRightY());
    } 
}