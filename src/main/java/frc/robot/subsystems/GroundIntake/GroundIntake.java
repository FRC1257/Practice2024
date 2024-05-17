package frc.robot.subsystems.GroundIntake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PID;
import frc.robot.util.LoggedTunableNumber;

public class GroundIntake extends SubsystemBase{
    private final GroundIntakeIO io;
    private GroundIntakeIOInputsAutoLogged inputs;

    public GroundIntake(GroundIntakeIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }

    /**
     * A command that runs the ground intake a certain speed (-1 <= speed <= 1) using PID
     * @param speedSupplier a function continuously returning the desired speed (in case it changes)
     */
    public Command runSpeedCommand(DoubleSupplier voltsSupplier) {
        return new FunctionalCommand(
            () -> {},
            () -> io.set(voltsSupplier.getAsDouble()),
            (interrupt) -> io.set(0),
            () -> false,
            this
        );
    }

    /**
     * A command that runs the ground intake a certain speed, in radians per second, using PID
     * @param speed the desired speed, in radians per second
     */
    public Command runVoltageCommand(double volts) {
        return runSpeedCommand(() -> volts);
    }
}