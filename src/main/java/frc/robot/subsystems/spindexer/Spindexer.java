package frc.robot.subsystems.spindexer;

import static frc.robot.subsystems.spindexer.SpindexerConstants.*;

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

  private int numJamTicks = 0;

  public Spindexer(SpindexerIO io) {
    this.io = io;
    state = SpindexerState.STOPPED;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);

    if (inputs.supplyCurrent >= jamSupplyCurrentThreshold) {
      numJamTicks++;
    } else {
      numJamTicks = 0;
    }
    Logger.recordOutput("Spindexer Num Jam Ticks", numJamTicks);
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

  public int getNumJamTicks() {
    return numJamTicks;
  }
}
