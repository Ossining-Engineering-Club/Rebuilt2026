package frc.robot.subsystems.shooterflywheels;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.shooterflywheels.ShooterFlywheelsConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class ShooterFlywheels extends SubsystemBase{
    public static enum ShooterFlywheelsState {
        SHOOT,
        REVERSE,
        STOPPED
    }

    private final ShooterFlywheelsIO io;
    private final ShooterFlywheelsIOInputsAutologged inputs = new ShooterFlywheelsIOInputsAutologged();

    private ShooterFlywheelsState state;

    /** Intake Rollers construction */
    public ShooterFlywheels(ShooterFlywheelsIO io) {
        this.io = io;
        state = ShooterFlywheelsState.STOPPED;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake Rollers", inputs);

        if (state == ShooterFlywheelsState.STOPPED) {
            io.setRollersMotorVoltage(0.0);
        }
    }

    /** Sets motor voltage to predefined RPM forward */
    public void startMotor() {
        state = ShooterFlywheelsState.SHOOT;
        io.setShooterMotorVoltage(
            
        );
    }

    /** Reverses Intake Rollers motor */
    public void reverseMotor() {
        state = ShooterFlywheelsState.REVERSE;
        //if (Constants.currentMode == Mode.SIM) {}
        io.setShooterMotorVoltage(ShooterFlywheelsConstants.reverseVoltage);
    }

    /** Stops motor */
    public void stopMotor() {
        state = ShooterFlywheelsState.STOPPED;
        io.setShooterMotorVoltage(0.0);
    }

    public ShooterFlywheelsState getState() {
        return state;
    }

    public void setState(ShooterFlywheelsState state) {
        this.state = state;
    }

    public Command intake() {
        return Commands.runOnce(() -> startMotor(), this);
    }

    public Command eject() {
        return Commands.runOnce(() -> reverseMotor(), this);
    }
}

