package frc.robot.subsystems.Limelight;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIOInputsAutoLogged;
import frc.robot.subsystems.Limelight.LimelightIO.LimelightIO;
import frc.robot.utils.ShooterUtils;

public class Limelight extends SubsystemBase{
   private LimelightIO m_limelightIO;
   private LimelightIOInputsAutoLogged m_limelightIOInputs = new LimelightIOInputsAutoLogged();

   public Limelight(LimelightIO limelightIO) {
        this.m_limelightIO = limelightIO;
   }

   public double getAngleOffset() {
        return m_limelightIO.getAngleOffset();
   }

   public double getDistance() {
        return m_limelightIO.getDistance();
   }

   @Override
   public void periodic() {
       m_limelightIO.updateInputs(m_limelightIOInputs);
       Logger.processInputs("Limelight", m_limelightIOInputs);
   }
   
}
