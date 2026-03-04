package frc.robot.subsystems.intakepivot;

import static frc.robot.subsystems.intakepivot.IntakePivotConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();
  private final ProfiledPIDController pid;

  private boolean usingPID = false;
  private int ticksSinceLastPID = 1000000;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
        pid =
            new ProfiledPIDController(
                kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        break;
      case SIM:
        pid =
            new ProfiledPIDController(
                simP,
                simI,
                simD,
                new TrapezoidProfile.Constraints(simMaxVelocity, simMaxAcceleration));
        break;
      case REPLAY:
        pid =
            new ProfiledPIDController(
                kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        break;
      default:
        pid =
            new ProfiledPIDController(
                kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        break;
    }

    io.updateInputs(inputs);
    pid.reset(inputs.angleRadians);

    pid.setTolerance(pidTolerance);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake Pivot", inputs);

    Logger.recordOutput("Intake Pivot Angle", getAngle());
    Logger.recordOutput("Intake Pivot Setpoint", pid.getSetpoint().position);

    if (ticksSinceLastPID >= 2) usingPID = false;
    else usingPID = true;
    ticksSinceLastPID++;

    if (!usingPID) pid.reset(getAngle());

    // Soft Limits
    // if (getAngle() <= minAngle && inputs.appliedVolts < 0) setVoltage(0);
    // if (getAngle() >= maxAngle && inputs.appliedVolts > 0) setVoltage(0);
  }

  public double getAngle() {
    return inputs.angleRadians;
  }

  public void runGoal(double angleGoal) {
    // if (angleGoal > maxAngle) angleGoal = maxAngle;
    // if (angleGoal < minAngle) angleGoal = minAngle;

    setVoltage(pid.calculate(getAngle(), angleGoal));

    ticksSinceLastPID = 0;
  }

  public boolean atGoal() {
    return pid.atGoal();
  }

  public void stop() {
    setVoltage(0);
  }

  public void setVoltage(double voltage) {
    // Soft Limits
    // if (getAngle() <= minAngle) voltage = Math.max(0, voltage);
    // if (getAngle() >= maxAngle) voltage = Math.min(0, voltage);

    io.setVoltage(voltage);
  }

  public void resetSimState() {
    io.resetSimState();
  }

  public Command goToAngle(double angleGoal) {
    return Commands.run(() -> runGoal(angleGoal), this).until(this::atGoal).finallyDo(() -> stop());
  }

  public Command extend() {
    return goToAngle(extendedAngle);
  }

  public Command retract() {
    return goToAngle(retractedAngle);
  }

  public Command upright() {
    return goToAngle(uprightAngle);
  }
}
