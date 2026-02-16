package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final ProfiledPIDController pid;
  private final SimpleMotorFeedforward feedforward;

  private boolean usingPID = false;
  private int ticksSinceLastPID = 1000000;

  private double prevSetpointVelocity = 0;

  public Turret(TurretIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
        pid =
            new ProfiledPIDController(
                kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        break;
      case SIM:
        pid =
            new ProfiledPIDController(
                simP,
                simI,
                simD,
                new TrapezoidProfile.Constraints(simMaxVelocity, simMaxAcceleration));
        feedforward = new SimpleMotorFeedforward(simS, simV, simA);
        break;
      case REPLAY:
        pid =
            new ProfiledPIDController(
                kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        break;
      default:
        pid =
            new ProfiledPIDController(
                kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        break;
    }

    io.updateInputs(inputs);
    pid.reset(inputs.angleRadians);

    pid.setTolerance(pidTolerance);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    Logger.recordOutput("Turret Angle", getAngle());
    Logger.recordOutput("Turret Setpoint", pid.getSetpoint().position);
    Logger.recordOutput("Turret Setpoint Velocity", pid.getSetpoint().velocity);

    if (ticksSinceLastPID >= 2) usingPID = false;
    else usingPID = true;
    ticksSinceLastPID++;

    if (!usingPID) {
      pid.reset(getAngle(), getAngularVelocity());
      prevSetpointVelocity = pid.getSetpoint().velocity;
    }

    // Soft Limits
    if (getAngle() <= minAngle && inputs.appliedVolts < 0) setVoltage(0);
    if (getAngle() >= maxAngle && inputs.appliedVolts > 0) setVoltage(0);
  }

  public double getAngle() {
    return inputs.angleRadians;
  }

  public double getAngularVelocity() {
    return inputs.angularVelocityRadPerSec;
  }

  public void runGoal(double angleGoal) {
    if (angleGoal > maxAngle || angleGoal < minAngle) {
      if ((angleGoal - maxAngle + 2 * Math.PI) % (2 * Math.PI)
          < (minAngle - angleGoal + 2 * Math.PI) % (2 * Math.PI)) {
        angleGoal = maxAngle;
      } else {
        angleGoal = minAngle;
      }
    }

    setVoltage(
        pid.calculate(getAngle(), angleGoal)
            + feedforward.calculateWithVelocities(
                prevSetpointVelocity, pid.getSetpoint().velocity));

    ticksSinceLastPID = 0;
    prevSetpointVelocity = pid.getSetpoint().velocity;
  }

  public boolean atGoal() {
    return pid.atGoal();
  }

  public void stop() {
    setVoltage(0);
  }

  public void setVoltage(double voltage) {
    // Soft Limits
    if (getAngle() <= minAngle) voltage = Math.max(0, voltage);
    if (getAngle() >= maxAngle) voltage = Math.min(0, voltage);

    io.setVoltage(voltage);
  }

  public void resetSimState() {
    io.resetSimState();
  }

  public Command trackAngle(DoubleSupplier angleSupplier) {
    return Commands.run(() -> runGoal(angleSupplier.getAsDouble()), this).finallyDo(() -> stop());
  }

  public Command goToAngle(double angleGoal) {
    return Commands.run(() -> runGoal(angleGoal), this).until(this::atGoal).finallyDo(() -> stop());
  }
}
