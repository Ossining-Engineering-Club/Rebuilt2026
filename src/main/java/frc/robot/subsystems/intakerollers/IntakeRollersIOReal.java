package frc.robot.subsystems.intakerollers;

import frc.robot.subsystems.intakerollers.IntakeRollersConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import static frc.robot.subsystems.intakepivot.IntakePivotConstants.encoderPositionFactor;
import static frc.robot.subsystems.intakepivot.IntakePivotConstants.encoderVelocityFactor;
import static frc.robot.subsystems.intakepivot.IntakePivotConstants.isInverted;
import static frc.robot.subsystems.intakerollers.IntakeRollersConstants.rollersMotorFreeCurrentLimit;
import static frc.robot.subsystems.intakerollers.IntakeRollersConstants.rollersMotorReduction;
import static frc.robot.subsystems.intakerollers.IntakeRollersConstants.rollersMotorStallCurrentLimit;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeRollersIOReal implements IntakeRollersIO {
    private final SparkMax intakeMotor;

    public IntakeRollersIOReal() {
        intakeMotor = new SparkMax(IntakeRollersConstants.intakeRollersCANID, MotorType.kBrushless);

        var rollersMotorConfig = new SparkMaxConfig();
        rollersMotorConfig.inverted(isInverted).idleMode(IdleMode.kBrake);
        rollersMotorConfig.smartCurrentLimit(rollersMotorStallCurrentLimit, rollersMotorFreeCurrentLimit);
        tryUntilOk(
                intakeMotor,
                5,
                () ->
                        rollersMotor.configure(
                                rollersMotorConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(IntakeRollersIOInputs inputs) {
        inputs.appliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.statorCurrent = intakeMotor.getOutputCurrent();
    }

    @Override
    public void setRollersMotorVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }
}
