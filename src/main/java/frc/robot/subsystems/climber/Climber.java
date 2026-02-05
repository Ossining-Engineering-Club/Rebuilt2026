package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    if (inputs.climberPosition >= ClimberConstants.maxPosition
        && inputs.climberMotorAppliedVolts > 0) stop();
    if (inputs.climberPosition <= ClimberConstants.minPosition
        && inputs.climberMotorAppliedVolts < 0) stop();
  }

  public void forward() {
    if (inputs.climberPosition < ClimberConstants.maxPosition) {
      io.setClimberMotorVoltage(ClimberConstants.climberMotorForwardVoltage);
    } else stop();
  }

  public void reverse() {
    if (inputs.climberPosition > ClimberConstants.minPosition) {
      io.setClimberMotorVoltage(ClimberConstants.climberMotorReverseVoltage);
    } else stop();
  }

  public void forwardClimb() {
    if (inputs.climberPosition < ClimberConstants.maxPosition) {
      io.setClimberMotorVoltage(ClimberConstants.climberMotorForwardVoltage);
    } else stop();
  }

  public void reverseClimb() {
    if (inputs.climberPosition > ClimberConstants.minPosition) {
      io.setClimberMotorVoltage(ClimberConstants.climberMotorReverseVoltage);
    } else stop();
  }

  public void stop() {
    io.setClimberMotorVoltage(0.0);
  }

  public double getPosition() {
    return inputs.climberPosition;
  }

  public Command extend() {
    return Commands.runOnce(() -> forward(), this)
        .andThen(Commands.waitUntil(() -> getPosition() >= ClimberConstants.extendPosition))
        .andThen(Commands.runOnce(() -> stop(), this));
  }

  public Command retract() {
    // return Commands.runOnce(() -> reverse(), this)
    //         .andThen(Commands.waitUntil(() -> getPosition() <= retractPosition))
    //         .andThen(Commands.runOnce(() -> stop(), this));
    return Commands.runOnce(() -> {});
  }

  public Command store() {
    return Commands.runOnce(() -> reverse(), this)
        .andThen(Commands.waitUntil(() -> getPosition() <= ClimberConstants.storePosition))
        .andThen(Commands.runOnce(() -> stop(), this));
  }
}
