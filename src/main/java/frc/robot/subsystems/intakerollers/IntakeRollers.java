package frc.robot.subsystems.intakerollers;

import static frc.robot.subsystems.intakerollers.IntakeRollersConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends SubsystemBase {
  public static enum IntakeRollersState {
    INTAKING,
    EJECTING,
    STOPPED
  }

  private final IntakeRollersIO io;
  private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

  private IntakeRollersState state;

  /** Intake Rollers construction */
  public IntakeRollers(IntakeRollersIO io) {
    this.io = io;
    state = IntakeRollersState.STOPPED;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake Rollers", inputs);
  }

  /** Sets motor voltage to predefined voltage forward */
  public void startMotor() {
    state = IntakeRollersState.INTAKING;
    io.setRollersMotorVoltage(IntakeRollersConstants.forwardVoltage);
  }

  /** Reverses Intake Rollers motor */
  public void reverseMotor() {
    state = IntakeRollersState.EJECTING;
    io.setRollersMotorVoltage(IntakeRollersConstants.reverseVoltage);
  }

  /** Stops motor */
  public void stopMotor() {
    state = IntakeRollersState.STOPPED;
    io.setRollersMotorVoltage(0.0);
  }

  public IntakeRollersState getState() {
    return state;
  }

  public void setState(IntakeRollersState state) {
    this.state = state;
  }

  public Command intake() {
    return Commands.runOnce(() -> startMotor(), this);
  }

  public Command eject() {
    return Commands.runOnce(() -> reverseMotor(), this);
  }
}
