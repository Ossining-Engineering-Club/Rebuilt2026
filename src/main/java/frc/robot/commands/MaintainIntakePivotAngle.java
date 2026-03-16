package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakepivot.IntakePivotConstants;

public class MaintainIntakePivotAngle extends Command {
  private final IntakePivot intakePivot;
  private final double angle;
  private boolean adjusting = false;

  public MaintainIntakePivotAngle(IntakePivot intakePivot, double angle) {
    this.intakePivot = intakePivot;
    this.angle = angle;

    addRequirements(intakePivot);
  }

  @Override
  public void initialize() {
    adjusting = false;
  }

  @Override
  public void execute() {
    if (Math.abs(intakePivot.getAngle() - angle) >= IntakePivotConstants.maintainAngleTolerance
        && intakePivot.getAngle() > angle) {
      adjusting = true;
    }
    if (intakePivot.atGoal() || intakePivot.getAngle() < angle) {
      adjusting = false;
    }

    if (adjusting) {
      intakePivot.runGoal(angle);
    } else {
      // intakePivot.stop();
      intakePivot.setVoltage(IntakePivotConstants.holdingIntakingAngleVoltage);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakePivot.stop();
  }
}
