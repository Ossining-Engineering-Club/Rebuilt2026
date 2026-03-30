package frc.robot.subsystems.intakepivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class IntakePivotConstants {
  // CAN ID
  public static final int canid = 20;

  // Control System
  public static final double kP = 20.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double maxVelocity = Units.degreesToRadians(360);
  public static final double maxAcceleration = Units.degreesToRadians(720);
  public static final double simP = 20.0;
  public static final double simI = 0.0;
  public static final double simD = 0.0;
  public static final double simMaxVelocity = Units.degreesToRadians(360);
  public static final double simMaxAcceleration = Units.degreesToRadians(720);
  public static final double pidTolerance = Units.degreesToRadians(0.5);
  public static final double maintainAngleTolerance = Units.degreesToRadians(1.0);

  // Angle Setpoints
  public static final double maxAngle = Units.degreesToRadians(108.0);
  public static final double minAngle = Units.degreesToRadians(9.5);
  public static final double startAngle = Units.degreesToRadians(108.0);
  public static final double extendedAngle = Units.degreesToRadians(9.5);
  // public static final double uprightAngle = Units.degreesToRadians(75.0);
  public static final double retractedAngle = Units.degreesToRadians(108.0);
  public static final double agitationTopAngle = Units.degreesToRadians(40.0);
  public static final double agitationBottomAngle = Units.degreesToRadians(9.5);

  public static final double agitationPeriodSeconds = 2;

  // Motor Constants
  public static final int stallCurrentLimit = 38;
  public static final int freeCurrentLimit = 38;
  public static final int supplyCurrentLimit = 38;
  public static final double motorReduction = 210.0 / 1.0; // 94.5 / 1.0;
  public static final boolean isInverted = false;
  public static final double encoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec
  public static final double sensorMechanismRatio =
      1 / (2 * Math.PI / motorReduction); // Rotor Rotations -> Radians

  // Simulator settings
  public static final DCMotor gearbox = DCMotor.getFalcon500(1);
  public static final double pivotMOI = 0.1375532206;
  public static final double intakeLengthMeters = Units.inchesToMeters(12.5);
}
