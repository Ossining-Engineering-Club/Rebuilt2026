package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  public static enum FeederState {
    FORWARD,
    REVERSE,
    STOPPED
  }

  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  private FeederState state;

  /** Feeder construction */
  public Feeder(FeederIO io) {
    this.io = io;
    state = FeederState.STOPPED;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  /** Sets motor voltage to predefined voltage forward */
  public void startMotor() {
    state = FeederState.FORWARD;
    io.setFeederMotorVoltage(FeederConstants.forwardVoltage);
  }

  /** Reverses feeder motor */
  public void reverseMotor() {
    state = FeederState.REVERSE;
    io.setFeederMotorVoltage(FeederConstants.reverseVoltage);
  }

  /** Stops motor */
  public void stopMotor() {
    state = FeederState.STOPPED;
    io.setFeederMotorVoltage(0.0);
  }

  public FeederState getState() {
    return state;
  }

  public void setState(FeederState state) {
    this.state = state;
  }
}
