package frc.robot.commands.subsystems.GroundIntake;

import static frc.robot.Constants.PID;

public interface GroundIntakeIO {
    @Autolog
    public static class GroundIntakeIOInputs {
        public double angleRad;
        public double angVelocityRadPerSec;
        public double appliedVolts;
        public double[] currentAmps;
        public double[] tempCelsius;
        public double setpointVelocity;
    }
    /** Updates the values found in the GroundIntakeIOInputs class (used for logging) */
    public default void updateInputs(GroundIntakeIOInputs inputs) {}

    /** sets the raw voltage of the motor (NO velocity PID) */
    public default void setVoltage(double voltage) {}

    /** sets the speed of the motor using velocity PID */
    public default void setSpeedRad(double speed) {}

    /** gets the angular velocity of the flywheels */
    public default double getSpeedRad() {return 0;}

    /** if true, this function will force the flywheel to break */
    public default void setBrake(boolean brake) {}

    /** sets the PID of the flywheel */
    public default void setPID(PID pid) {}
}