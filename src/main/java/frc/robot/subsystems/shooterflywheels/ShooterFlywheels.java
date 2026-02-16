package frc.robot.subsystems.shooterflywheels;

import static frc.robot.subsystems.shooterflywheels.ShooterFlywheelsConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterFlywheels extends SubsystemBase {
  public static enum ShooterFlywheelsState {
    FORWARD,
    REVERSE,
    STOPPED
  }

  private final ShooterFlywheelsIO io;
  private final ShooterFlywheelsIOInputsAutoLogged inputs =
      new ShooterFlywheelsIOInputsAutoLogged();

  private ShooterFlywheelsState state;

  /** Shooter Flywheels construction */
  public ShooterFlywheels(ShooterFlywheelsIO io) {
    this.io = io;
    state = ShooterFlywheelsState.STOPPED;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter Flywheels", inputs);
  }

  /** Sets RPM of flywheel for velocity control */
  public void setRPM(double RPM) {
    if (RPM == 0.0) {
      // if requested RPM is 0, just set the voltage to 0 rather than using velocity control
      setState(ShooterFlywheelsState.STOPPED);
      stop();
    } else {
      if (RPM > 0.0) setState(ShooterFlywheelsState.FORWARD);
      else setState(ShooterFlywheelsState.REVERSE);
      io.setRPM(RPM);
    }
  }

  /** Stops flywheel */
  public void stop() {
    state = ShooterFlywheelsState.STOPPED;
    io.setVoltage(0.0);
  }

  /** Sets voltage of flywheel motors */
  public void setVoltage(double voltage) {
    if (voltage == 0.0) setState(ShooterFlywheelsState.STOPPED);
    else if (voltage > 0.0) setState(ShooterFlywheelsState.FORWARD);
    else setState(ShooterFlywheelsState.REVERSE);
    io.setVoltage(voltage);
  }

  public ShooterFlywheelsState getState() {
    return state;
  }

  private void setState(ShooterFlywheelsState state) {
    this.state = state;
  }
}
