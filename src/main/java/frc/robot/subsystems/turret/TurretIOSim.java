package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

public class TurretIOSim implements TurretIO {
  private final DCMotorSim turretMotorSim;
  private double appliedVolts = 0.0;

  public TurretIOSim() {
    turretMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, moi, motorReduction), gearbox);

    turretMotorSim.setState(0, 0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    turretMotorSim.update(0.02);

    inputs.appliedVolts = appliedVolts;
    inputs.angleRadians = turretMotorSim.getAngularPositionRad();
    inputs.angularVelocityRadPerSec = turretMotorSim.getAngularVelocityRadPerSec();
    inputs.absEncoder18tReading = Units.radiansToRotations(inputs.angleRadians) * AE18tReduction;
    inputs.absEncoder19tReading = Units.radiansToRotations(inputs.angleRadians) * AE19tReduction;
    inputs.absAngleRadians =
        (inputs.absEncoder18tReading - inputs.absEncoder19tReading) * AEDifferenceMultiplier;
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    Logger.recordOutput("SetVoltages/Turret", appliedVolts);
    turretMotorSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void resetSimState() {
    turretMotorSim.setState(0, 0);
  }
}
