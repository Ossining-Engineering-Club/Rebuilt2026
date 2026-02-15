package frc.robot.subsystems.intakepivot;

import static frc.robot.subsystems.intakepivot.IntakePivotConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;

public class IntakePivotIOSim implements IntakePivotIO {
  public final SingleJointedArmSim intakePivotSim;
  public double appliedVolts = 0.0;

  public IntakePivotIOSim() {
    intakePivotSim =
        new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(gearbox, pivotMOI, motorReduction),
            gearbox,
            motorReduction,
            intakeLengthMeters,
            minAngle,
            maxAngle,
            false,
            startAngle);

    intakePivotSim.setState(startAngle, 0.0);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    intakePivotSim.update(0.02);

    inputs.appliedVolts = appliedVolts;
    inputs.angleRadians = intakePivotSim.getAngleRads();
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    Logger.recordOutput("SetVoltages/IntakePivot", appliedVolts);
    intakePivotSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void resetSimState() {
    intakePivotSim.setState(startAngle, 0.0);
  }
}
