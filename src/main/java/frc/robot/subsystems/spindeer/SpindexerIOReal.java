package frc.robot.subsystems.spindeer;

import static frc.robot.subsystems.spindeer.SpindexerConstants.*;

import frc.robot.subsystems.spindeer.SpindexerIO.SpindexerIOInputs;
import frc.robot.util.SparkUtil;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;

import static frc.robot.subsystems.spindeer.SpindexerIO.*;

public class SpindexerIOReal {
    private final  ///// spindexMotor;

    public IntakeRollersIOReal() {
        this.spindexMotor = new TalonFX(SpindexerConstants.spindexerCanID, MotorType.kBrushless);

        var spindexMotorConfig = new TalonFXConfiguration();


        SparkUtil.tryUntilOk(
            spindexMotor,
            5,
             (null) -> 
             
                        spindexMotor.configure(
                            spindexMotorConfig,
                            ResetMode.kResetSafeParameters,
                            PersistMode.kPersistParameters
                        ));
    }

    
    public void updateInputs(SpindexerIOInputs inputs) {

        inputs.appliedVolts = spindexMotor.getAppliedOutput() * spindexMotor.getBusVoltage();
        inputs.statorCurrent = spindexMotor.getOutputCurrent();

    

    }
    
    public void setRollersMotorVoltage(double voltage) {
        spindexMotor.setVoltage(voltage);
    }
}
