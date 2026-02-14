package frc.robot.subsystems.intakepivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class IntakePivotConstants {
  // CAN ID
  public static final int canid = 0;

  // Control System
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double maxVelocity = Units.degreesToRadians(0);
  public static final double maxAcceleration = Units.degreesToRadians(0);
  public static final double simP = 0.0;
  public static final double simI = 0.0;
  public static final double simD = 0.0;
  public static final double simMaxVelocity = Units.degreesToRadians(0);
  public static final double simMaxAcceleration = Units.degreesToRadians(0);
  public static final double pidTolerance = Units.degreesToRadians(0);

  // Angle Setpoints
  public static final double maxAngle = Units.degreesToRadians(114.0);
  public static final double minAngle = Units.degreesToRadians(1.0);
  public static final double startAngle = Units.degreesToRadians(114.0);

  // Motor Constants
  public static final int stallCurrentLimit = 38;
  public static final int freeCurrentLimit = 38;
  public static final double motorReduction = 94.5 / 1.0;
  public static final boolean isInverted = false;
  public static final double encoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec
  public static final double sensorMechanismRatio =
      1 / (2 * Math.PI / motorReduction); // Rotor Rotations -> Radians

  // Simulator settings
  public static final DCMotor gearbox = DCMotor.getNEO(1);
  public static final double pivotMOI = 0.1375532206;
  public static final double intakeLengthMeters = Units.inchesToMeters(12.5);
}
