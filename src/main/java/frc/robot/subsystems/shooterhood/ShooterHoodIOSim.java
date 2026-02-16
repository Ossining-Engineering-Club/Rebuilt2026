package frc.robot.subsystems.shooterhood;

import static frc.robot.subsystems.shooterhood.ShooterHoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

public class ShooterHoodIOSim implements ShooterHoodIO {
  public final DCMotorSim shooterHoodSim;
  public double appliedVolts = 0.0;

  public ShooterHoodIOSim() {
    shooterHoodSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, hoodMOI, motorReduction), gearbox);

    shooterHoodSim.setState(startAngle, 0.0);
  }

  @Override
  public void updateInputs(ShooterHoodIOInputs inputs) {
    shooterHoodSim.update(0.02);

    inputs.appliedVolts = appliedVolts;
    inputs.angleRadians = shooterHoodSim.getAngularPositionRad();
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    Logger.recordOutput("SetVoltages/ShooterHood", appliedVolts);
    shooterHoodSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void resetSimState() {
    shooterHoodSim.setState(startAngle, 0.0);
  }
}
