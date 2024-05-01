package frc.robot.util.Exceptions;

public class VoltageToleranceException extends Exception {
    public VoltageToleranceException(String ErrorMessage) {
        super(ErrorMessage);
    }
}