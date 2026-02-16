package frc.robot.subsystems.shooterflywheels;

import static frc.robot.subsystems.shooterflywheels.ShooterFlywheelsConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterFlywheelsIOSim implements ShooterFlywheelsIO {
  private final FlywheelSim flywheelSim;
  private final PIDController pid = new PIDController(simP, simI, simD);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(simS, simV);
  private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  public ShooterFlywheelsIOSim() {
    flywheelSim =
        new FlywheelSim(LinearSystemId.createFlywheelSystem(gearbox, moi, motorReduction), gearbox);
  }

  @Override
  public void updateInputs(ShooterFlywheelsIOInputs inputs) {
    flywheelSim.update(0.02);

    if (closedLoop) {
      appliedVolts =
          pid.calculate(flywheelSim.getAngularVelocityRPM())
              + feedforward.calculate(flywheelSim.getAngularVelocityRPM());
    } else {
      pid.reset();
    }

    flywheelSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));

    inputs.RPM = flywheelSim.getAngularVelocityRPM();
    inputs.leftAppliedVolts = appliedVolts;
    inputs.rightAppliedVolts = appliedVolts;
  }

  @Override
  public void setRPM(double RPM) {
    closedLoop = true;
    pid.setSetpoint(RPM);
  }

  @Override
  public void setVoltage(double voltage) {
    closedLoop = false;
    appliedVolts = voltage;
  }
}
