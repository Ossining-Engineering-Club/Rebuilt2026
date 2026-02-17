package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.Feeder.FeederState;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakepivot.IntakePivotConstants;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.intakerollers.IntakeRollers.IntakeRollersState;
import frc.robot.subsystems.shooterflywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterflywheels.ShooterFlywheels.ShooterFlywheelsState;
import frc.robot.subsystems.shooterhood.ShooterHood;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.Spindexer.SpindexerState;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.FuelSim;
import org.littletonrobotics.junction.AutoLogOutput;

public class SimulationManager {
  private final int FUEL_CARRYING_CAPACITY = 60;
  private final double SHOOTER_EXIT_VELOCITY_SCALAR =
      0.8; // the tangential velocity of the flywheels are multiplied by this scalar to calculate
  // the exit velocity of fuel
  private final double BALLS_PER_SECOND = 7;

  private final FuelSim fuelSim;
  private final ShooterFlywheels shooterFlywheels;
  private final ShooterHood shooterHood;
  private final IntakePivot intakePivot;
  private final IntakeRollers intakeRollers;
  private final Spindexer spindexer;
  private final Turret turret;
  private final Feeder feeder;

  @AutoLogOutput(key = "FuelCount")
  private int fuelCount = 8; // stores how many fuel the robot currently has

  private double timeOfNextShot = -1;
  private boolean shotQueued = false;

  public SimulationManager(
      FuelSim fuelSim,
      ShooterFlywheels shooterFlywheels,
      ShooterHood shooterHood,
      IntakePivot intakePivot,
      IntakeRollers intakeRollers,
      Spindexer spindexer,
      Turret turret,
      Feeder feeder) {
    this.fuelSim = fuelSim;
    this.shooterFlywheels = shooterFlywheels;
    this.shooterHood = shooterHood;
    this.intakePivot = intakePivot;
    this.intakeRollers = intakeRollers;
    this.spindexer = spindexer;
    this.turret = turret;
    this.feeder = feeder;
  }

  public void periodic() {
    // Check whether a simulated ball should be shot
    if (Timer.getFPGATimestamp() >= timeOfNextShot && shotQueued) {
      if (isShooting()) {
        shootBall();
        timeOfNextShot += 1.0 / BALLS_PER_SECOND;
      } else {
        shotQueued = false;
      }
    } else if (!shotQueued && isShooting()) {
      shootBall();
      timeOfNextShot = Timer.getFPGATimestamp() + 1.0 / BALLS_PER_SECOND;
      shotQueued = true;
    }
  }

  private void shootBall() {
    fuelSim.launchFuel(
        MetersPerSecond.of(
            shooterFlywheels.getRPM()
                / 60.0
                * Math.PI
                * Units.inchesToMeters(4)
                * SHOOTER_EXIT_VELOCITY_SCALAR),
        Radians.of(shooterHood.getAngle()),
        Radians.of(turret.getAngle()),
        Constants.shooterOffset);
    fuelCount--;
  }

  @AutoLogOutput(key = "isShooting")
  private boolean isShooting() {
    return shooterFlywheels.getState() == ShooterFlywheelsState.FORWARD
        && feeder.getState() == FeederState.FORWARD
        && spindexer.getState() == SpindexerState.FORWARD
        && fuelCount > 0;
  }

  @AutoLogOutput(key = "isIntaking")
  public boolean isIntaking() {
    return withinTolerance(
            intakePivot.getAngle(),
            IntakePivotConstants.extendedAngle,
            IntakePivotConstants.pidTolerance)
        && intakeRollers.getState() == IntakeRollersState.INTAKING
        && fuelCount < FUEL_CARRYING_CAPACITY;
  }

  public void incrementFuelCount() {
    fuelCount++;
  }

  private boolean withinTolerance(double a, double b, double tolerance) {
    return Math.abs(a - b) <= tolerance;
  }
}
