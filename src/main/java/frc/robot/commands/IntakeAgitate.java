package frc.robot.commands;

import static frc.robot.subsystems.intakepivot.IntakePivotConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakepivot.IntakePivot;

// Agitate hopper by moving intake pivot following a cosine curve
public class IntakeAgitate extends Command {
  private final IntakePivot intakePivot;

  private final double a = (agitationTopAngle - agitationBottomAngle) / 2.0;
  private final double b = 2 * Math.PI / agitationPeriodSeconds;
  private int tickCounter = 0;

  public IntakeAgitate(IntakePivot intakePivot) {
    this.intakePivot = intakePivot;

    addRequirements(intakePivot);
  }

  @Override
  public void initialize() {
    tickCounter = 0;
  }

  @Override
  public void execute() {
    intakePivot.runGoal(-a * Math.cos(b * (tickCounter / 50.0)) + agitationBottomAngle + a);
    tickCounter++;
  }

  @Override
  public void end(boolean interrupted) {
    // CommandScheduler.getInstance().schedule(intakePivot.goToAngle(extendedAngle));
    intakePivot.stop();
  }
}
