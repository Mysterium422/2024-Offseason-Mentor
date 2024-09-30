package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberIO.ClimberIO;
import frc.robot.subsystems.Climber.ClimberIO.ClimberIOInputsAutoLogged;;

public class Climber extends SubsystemBase{
    private CommandXboxController m_operatorController;    
    private ClimberIO climberIO;
    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(CommandXboxController operator, ClimberIO climberIO) {
        this.m_operatorController = operator;
        this.climberIO = climberIO;
    }

    @Override
    public void periodic() {        
        climberIO.setPower(Constants.deadzone(-m_operatorController.getRightY()), Constants.deadzone(-m_operatorController.getLeftY()));
        climberIO.updateInputs(inputs);
        Logger.recordOutput("leftOpY", m_operatorController.getLeftY());
        Logger.recordOutput("rightOpY", m_operatorController.getRightY());
    } 
}
