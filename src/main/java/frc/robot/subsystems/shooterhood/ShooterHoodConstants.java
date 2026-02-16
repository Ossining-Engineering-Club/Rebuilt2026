package frc.robot.subsystems.shooterhood;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ShooterHoodConstants {
  // CAN ID
  public static final int shooterHoodCANID = 0;

  // Motor Constants
  public static final int stallCurrentLimit = 38;
  public static final int freeCurrentLimit = 38;
  public static final boolean isInverted = false;
  public static final double encoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec
  public static final double motorReduction = 418.5 / 1.0;

  // Control System
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final double maxVelocity = Units.degreesToRadians(0);
  public static final double maxAcceleration = Units.degreesToRadians(0);
  public static final double simP = 100.0;
  public static final double simI = 0.0;
  public static final double simD = 0.0;
  public static final double simS = 0.0;
  public static final double simV = 8.1;
  public static final double simA = 0.0;
  public static final double simMaxVelocity = Units.degreesToRadians(360);
  public static final double simMaxAcceleration = Units.degreesToRadians(720);
  public static final double pidTolerance = Units.degreesToRadians(0.25);

  // Angle Setpoints
  public static final double maxAngle = Units.degreesToRadians(66);
  public static final double minAngle = Units.degreesToRadians(40);
  public static final double startAngle = Units.degreesToRadians(66);

  // Sim settings
  public static final DCMotor gearbox = DCMotor.getNeoVortex(1);
  public static final double hoodMOI = 0.0345443073;
}
