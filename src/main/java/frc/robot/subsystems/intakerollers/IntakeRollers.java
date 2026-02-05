package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.intakerollers.IntakeRollersConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends SubsystemBase{
    public static enum IntakeRollersState {
        INTAKE,
        EJECT,
        STOPPED
    }

    private final IntakeRollersIO io;
    private final IntakeRollersIOInputsAutologged inputs = new IntakeRollersIOInputsAutologged();

    private IntakeRollersState state;

    /** Intake Rollers construction */
    public IntakeRollers(IntakeRollersIO io) {
        this.io = io;
        state = IntakeRollersState.STOPPED;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake Rollers", inputs);

        if (state == IntakeRollersState.STOPPED) {
            io.setRollersMotorVoltage(0.0);
        }
    }

    /** Sets motor voltage to predefined voltage forward */
    public void startMotor() {
        state = IntakeRollersState.INTAKE;
        io.setRollersMotorVoltage(IntakeRollersConstants.forwardVoltage);
    }

    /** Reverses Intake Rollers motor */
    public void reverseMotor() {
        state = IntakeRollersState.EJECT;
        //if (Constants.currentMode == Mode.SIM) {}
        io.setRollersMotorVoltage(IntakeRollersConstants.reverseVoltage);
    }

    /** Stops motor */
    public void stopMotor() {
        state = IntakeRollersState.STOPPED;
        io.setRollersMotorVoltage(0.0);
    }

    public IntakeRollersState getState() {
        return state;
    }

    public void setState(IntakeRollersState state) {
        this.state = state;
    }

    public Command intake() {
        return Commands.runOnce(() -> startMotor(), this);
    }

    public Command eject() {
        return Commands.runOnce(() -> reverseMotor(), this);
    }
}


