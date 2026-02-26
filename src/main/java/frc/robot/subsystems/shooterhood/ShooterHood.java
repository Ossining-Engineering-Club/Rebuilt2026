package frc.robot.subsystems.shooterhood;

import static frc.robot.subsystems.shooterhood.ShooterHoodConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterHood extends SubsystemBase {
  private final ShooterHoodIO io;
  private final ShooterHoodIOInputsAutoLogged inputs = new ShooterHoodIOInputsAutoLogged();
  private final ProfiledPIDController
      mainPID; // main PID generates the motion profile and handles tolerances
  private final PIDController
      secondaryPID; // secondary PID calculates the feedback output using the main PID's motion
  // profile
  private final SimpleMotorFeedforward feedforward;

  private boolean usingPID = false;
  private int ticksSinceLastPID = 1000000;

  private double prevSetpointVelocity = 0;

  public ShooterHood(ShooterHoodIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
        mainPID =
            new ProfiledPIDController(
                0, 0, 0, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        secondaryPID = new PIDController(kP, kI, kD);
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        break;
      case SIM:
        mainPID =
            new ProfiledPIDController(
                0, 0, 0, new TrapezoidProfile.Constraints(simMaxVelocity, simMaxAcceleration));
        secondaryPID = new PIDController(simP, simI, simD);
        feedforward = new SimpleMotorFeedforward(simS, simV, simA);
        break;
      case REPLAY:
        mainPID =
            new ProfiledPIDController(
                0, 0, 0, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        secondaryPID = new PIDController(kP, kI, kD);
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        break;
      default:
        mainPID =
            new ProfiledPIDController(
                0, 0, 0, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        secondaryPID = new PIDController(kP, kI, kD);
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        break;
    }

    io.updateInputs(inputs);
    mainPID.reset(inputs.angleRadians);

    mainPID.setTolerance(pidTolerance);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter Hood", inputs);

    Logger.recordOutput("Shooter Hood Angle", getAngle());
    Logger.recordOutput("Shooter Hood Setpoint", mainPID.getSetpoint().position);
    Logger.recordOutput("Shooter Hood Setpoint Velocity", mainPID.getSetpoint().velocity);

    if (ticksSinceLastPID >= 2) usingPID = false;
    else usingPID = true;
    ticksSinceLastPID++;

    if (!usingPID) {
      mainPID.reset(getAngle());
      prevSetpointVelocity = mainPID.getSetpoint().velocity;
    }

    // Soft Limits
    // if (getAngle() <= minAngle && inputs.appliedVolts < 0) setVoltage(0);
    // if (getAngle() >= maxAngle && inputs.appliedVolts > 0) setVoltage(0);
  }

  public double getAngle() {
    return inputs.angleRadians;
  }

  public void runGoal(double angleGoal) {
    if (angleGoal > maxAngle) angleGoal = maxAngle;
    if (angleGoal < minAngle) angleGoal = minAngle;

    double pidOutput = secondaryPID.calculate(getAngle(), mainPID.getSetpoint().position);
    mainPID.calculate(getAngle(), angleGoal);

    setVoltage(
        pidOutput
            + feedforward.calculateWithVelocities(
                prevSetpointVelocity, mainPID.getSetpoint().velocity));

    ticksSinceLastPID = 0;
    prevSetpointVelocity = mainPID.getSetpoint().velocity;
  }

  public boolean atGoal() {
    return mainPID.atGoal();
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

  public Command trackAngle(DoubleSupplier angleSupplier) {
    return Commands.run(() -> runGoal(angleSupplier.getAsDouble()), this).finallyDo(() -> stop());
  }

  public Command goToAngle(double angleGoal) {
    return Commands.run(() -> runGoal(angleGoal), this).until(this::atGoal).finallyDo(() -> stop());
  }
}
