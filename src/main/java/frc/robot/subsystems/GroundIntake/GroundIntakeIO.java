package frc.robot.subsystems.GroundIntake;

import static frc.robot.Constants.PID;

import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {
    @AutoLog
    public static class GroundIntakeIOInputs {
        public double angVelocityRadPerSec;
        public double desiredVolts;
        public double appliedVolts;
        public double currentAmps;
        public double tempCelsius;
        public boolean isVoltageClose;
    }
    /** Updates the values found in the GroundIntakeIOInputs class (used for logging) */
    public default void updateInputs(GroundIntakeIOInputs inputs) {}

    /** sets the raw speed of the motor (-1 <= speed <= 1) */
    public default void set(double speed) {}

    /** gets the angular velocity of the flywheels */
    public default double getAngVelocity() { return 0; }

    /** if true, this function will force the flywheel to break */
    public default void setBrake(boolean brake) {}
}