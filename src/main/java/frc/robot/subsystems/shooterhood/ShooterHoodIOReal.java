package frc.robot.subsystems.shooterhood;

import static frc.robot.subsystems.shooterhood.ShooterHoodConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.subsystems.intakepivot.IntakePivotIO;

public class ShooterHoodIOReal implements ShooterHoodIO {
    private final SparkMax hoodMotor;
    private final RelativeEncoder encoder;

    public ShooterHoodIOReal() {
        hoodMotor = new SparkMax(shooterHoodCANID, MotorType.kBrushless);
        encoder = sparkMax.getEncoder();
    }
}
