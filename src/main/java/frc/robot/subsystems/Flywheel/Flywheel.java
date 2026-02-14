package frc.robot.subsystems.Flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOInputsAutoLogged;


public class Flywheel extends SubsystemBase {
    
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    
    public Flywheel(Flywheel io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
      }

  
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void stop() {
    io.setVoltage(0.0);
  }


  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  public boolean isSpunUp() {
    return Math.abs(getVelocityRPM() - FlywheelConstants.kDefaultRPM)
        <= FlywheelConstants.kRPMTolerance;
  }

 
}

    


