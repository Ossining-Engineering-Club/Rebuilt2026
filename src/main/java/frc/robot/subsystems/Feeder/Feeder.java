package main.java.frc.robot.subsystems.Feeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase{
    public static enum FeederState {
        INTAKE,
        EJECT,
        STOPPED
    }

    private final FeederIO io;
    private final FeederIOInputsAutologged inputs = new FeederIOInputsAutologged();

    private  FeederState state;

    /** Kicker construction */
    public Feeder(FeederIO io) {
        this.io = io;
        state = FeederState.STOPPED;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Kicker", inputs);

        if (state == FeederState.STOPPED) {
            io.setFeederMotorVoltage(0.0);
        }
    }

    /** Sets motor voltage to predefined voltage forward */
    public void startMotor() {
        state = FeederState.INTAKE;
        io.setFeederMotorVoltage(FeederConstants.forwardVoltage);
    }

    /** Reverses Kicker motor */
    public void reverseMotor() {
        state = FeederState.EJECT;
        //if (Constants.currentMode == Mode.SIM) {}
        io.setFeederMotorVoltage(FeederConstants.reverseVoltage);
    }

    /** Stops motor */
    public void stopMotor() {
        state = FeederState.STOPPED;
        io.setFeederMotorVoltage(0.0);
    }

    public FeederState getState() {
        return state;
    }

    public void setState(FeederState state) {
        this.state = state;
    }
} 