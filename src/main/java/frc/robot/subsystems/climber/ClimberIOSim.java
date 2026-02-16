package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

public class ClimberIOSim implements ClimberIO {
  public final DCMotorSim climberSim;
  public double appliedVolts = 0.0;

  public ClimberIOSim() {
    climberSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ClimberConstants.gearbox,
                ClimberConstants.climberMOI,
                ClimberConstants.climberMotorReduction),
            ClimberConstants.gearbox);

    climberSim.setState(ClimberConstants.startPosition, 0.0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    climberSim.update(0.02);

    inputs.climberMotorAppliedVolts = appliedVolts;
    inputs.climberMotorAngleRadians = climberSim.getAngularPositionRad();
  }

  @Override
  public void setClimberMotorVoltage(double voltage) {
    appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    Logger.recordOutput("SetVoltages/Climber", appliedVolts);
    climberSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void resetSimState() {
    climberSim.setState(ClimberConstants.startPosition, 0.0);
  }
}
