package frc.robot.subsystems.intakepivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class IntakePivotConstants {

//MOST VALUES OF 0.0 ARE PLACEHOLDERS
    //CAN ID
    public static final int canid = 0;

    //Motor Voltages
    public static final double extendVoltage = 3.0;
    public static final double retractVoltage = -3.0;

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

    //Motor Constants
    public static final int stallCurrentLimit = 0;
    public static final int freeCurrentLimit = 0;
    public static final int currentLimit = 30;
    public static final double motorReduction = 0.0;
    public static final boolean isInverted = false;
    public static final double encoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec
    public static final double sensorMechanismRatio = 1 / (2 * Math.PI / motorReduction); // Rotor Rotations -> Radians
    
    //Simulator settings
    public static final DCMotor gearbox = DCMotor.getNeoVortex(1);
    public static final double pivotMOI = 0.0;
}