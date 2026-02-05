package frc.robot.subsystems.shooterhood;

import edu.wpi.first.math.util.Units;

public class ShooterHoodConstants {
    
//MOST VALUES OF 0.0 ARE PLACEHOLDERS
    //CAN ID
    public static final int shooterHoodCANID = 2;

    //Motor Power Level
    public static final double increaseAngleVoltage = 1.0;
    public static final double decreaseAngleVoltage = -1.0;

    //Motor Constants
    public static final int currentLimit = 30;
    public static final boolean isInverted = false;
    public static final double encoderPositionFactor = 1.0; // Rotations -> Rotations
    public static final double encoderVelocityFactor = 1.0 / 60; // RPM -> Rot/Sec

    //Control System
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

    //Angle Setpoints
    public static final double maxAngle = Units.degreesToRadians(0.0);
    public static final double minAngle = 0.0;
    public static final double startAngle = 0.0;
}
