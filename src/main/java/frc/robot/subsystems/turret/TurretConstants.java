package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class TurretConstants {
  // motor constants
  public static final int canId = 0;
  public static final double motorReduction = 51.75 / 1.0; // 143.75 / 1.0;
  public static final double encoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec
  public static final boolean isInverted = false;
  public static final int stallCurrentLimit = 38;
  public static final int freeCurrentLimit = 38;

  // control system
  public static final double kP = 0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final double maxVelocity = Units.degreesToRadians(360.0);
  public static final double maxAcceleration = Units.degreesToRadians(720.0);
  public static final double simP = 50;
  public static final double simI = 0;
  public static final double simD = 0;
  public static final double simS = 0.0;
  public static final double simV = 0.86;
  public static final double simA = 0.003;
  public static final double simMaxVelocity = Units.degreesToRadians(800);
  public static final double simMaxAcceleration = Units.degreesToRadians(3600);
  public static final double pidTolerance = Units.degreesToRadians(0.5);

  // angle limits
  public static final double maxAngle = Units.degreesToRadians(120.0);
  public static final double minAngle = Units.degreesToRadians(-30.0);
  public static final double storedAngle = 0;

  // absolute encoders
  public static final int AE18tChannel = 0;
  public static final int AE19tChannel = 1;
  public static final double AE18tReduction = (138.0 / 24.0) * (40.0 / 18.0);
  public static final double AE19tReduction = (138.0 / 24.0) * (40.0 / 19.0);
  public static final double AE18tOffsetRotations = 0;
  public static final double AE19tOffsetRotations = 0;
  public static final double AEDifferenceMultiplier =
      1.0
          / (AE18tReduction - AE19tReduction)
          * 2
          * Math
              .PI; // difference in the absolute encoder readings gets multiplied by this scalar to
  // calculate the turret's angle in radians

  // sim settings
  public static final DCMotor gearbox = DCMotor.getNeoVortex(1);
  public static final double moi = 0.087791896;
}
