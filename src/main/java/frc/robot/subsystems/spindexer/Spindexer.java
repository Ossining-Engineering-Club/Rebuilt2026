package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {

  public static enum SpindexerState {
    FORWARD,
    REVERSE,
    STOPPED
  }

  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
  private SpindexerState state;

  public Spindexer(SpindexerIO io) {
    this.io = io;
    state = SpindexerState.STOPPED;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
  }

  public void startMotor() {
    state = SpindexerState.FORWARD;
    io.setSpindexerMotorVoltage(SpindexerConstants.forwardVoltage);
  }

  public void stopMotor() {
    state = SpindexerState.STOPPED;
    io.setSpindexerMotorVoltage(0);
  }

  public void reverseMotor() {
    state = SpindexerState.REVERSE;
    io.setSpindexerMotorVoltage(SpindexerConstants.reverseVoltage);
  }

  public SpindexerState getState() {
    return state;
  }

  public void setState(SpindexerState state) {
    this.state = state;
  }
}
