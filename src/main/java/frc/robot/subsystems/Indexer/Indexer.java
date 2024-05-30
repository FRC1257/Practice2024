package frc.robot.subsystems.Indexer;

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

public class Indexer extends SubsystemBase{
    private final IndexerIO io;
    private IndexerIOInputsAutoLogged inputs;

    public Indexer(IndexerIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }

    @AutoLogOutput(key = "Indexer/isIntaked")
    public boolean isIntaked() {
        return io.isIntaked();
    }

    /**
     * A command that runs the ground intake a certain speed (-1 <= speed <= 1) using PID
     * @param speedSupplier a function continuously returning the desired speed (in case it changes)
     */
    public Command runSpeedCommand(DoubleSupplier speedSupplier) {
        return new FunctionalCommand(
            () -> {},
            () -> io.set(speedSupplier.getAsDouble()),
            (interrupt) -> io.set(0),
            () -> false,
            this
        );
    }

    /**
     * A command that runs the ground intake a certain speed (-1 <= speed <= 1) using PID
     * @param speed the desired speed
     */
    public Command runSpeedCommand(double speed) {
        return runSpeedCommand(() -> speed);
    }
}