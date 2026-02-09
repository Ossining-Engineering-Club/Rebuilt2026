package frc.robot.subsystems.spindeer;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.spindeer.SpindexerIO;

public class Spindexer extends SubsystemBase{

    public static enum SpindexerState {
        CLOCKWISE,
        COUNTERCLOCKWISE,
        STOPPED

    }

    private final SpindexerIO io;
    private final SpindexerIOInputsAutologged inputs = new SpindexerIOInputsAutologged();
    private SpindexerState state;

    public SpindexerState(SpindexerIO io) {
        this.io = io;
        state = SpindexerState.STOPPED;
    }

    @Override

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Spindexer Rollers", inputs);

        if (state == SpindexerState.STOPPED){
            io.setSpindexerMotorVoltage(0.0);
        }
    }

    public void startMotor() {
        state = SpindexerState.COUNTERCLOCKWISE;
        io.setSpindexerMotorVoltage(SpindexerConstants.counterclockwiseVoltage);
    }

    public void stopMotor() {
        state = SpindexerState.STOPPED;
        io.setSpindexerMotorVoltage(0);
    }

    public SpindexerState getState(){
        return state;
    }

    public void setState(SpindexerState state) {
        this.state = state;
    }

    public Command counterclockwise(){
        return Commands.runOnce(()-> startMotor(), this);
    }


}
 