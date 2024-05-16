package frc.robot.subsystems.GroundIntake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class GroundIntake extends SubsystemBase{
    private final GroundIntakeIO io;
    private final GroundIntakeIOInputsAutoLogged inputs;

    private LoggedTunableNumber kP;
    private LoggedTunableNumber kI;
    private LoggedTunableNumber kD;

    public GroundIntake(GroundIntakeIO io) {
        this.io = io;

        kP = new LoggedTunableNumber("GroundIntake/kP", io.getPID().kP());
        kI = new LoggedTunableNumber("GroundIntake/kI", io.getPID().kI());
        kD = new LoggedTunableNumber("GroundIntake/kD", io.getPID().kD());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command runSpeedCommand(DoubleSupplier speedSupplier) {
        return new FunctionalCommand(
            () -> {},
            () -> io.setSpeedPID(speedSupplier.getAsDouble()),
            (interrupt) -> {},
            () -> false,
            this
        );
    }

    public Command runSpeedCommand(double speed) {
        return runSpeedCommand(() -> speed);
    }
}