package frc.robot.subsystems.GroundIntake;

import static frc.robot.Constants.PID;

import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {
    @AutoLog
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

    /** gets the angular velocity of the flywheels */
    public default double getAngVelocity() {return 0;}

    /** if true, this function will force the flywheel to break */
    public default void setBrake(boolean brake) {}

    /** sets the PID of the flywheel */
    public default void setPID(PID pid) {}

    /** Returns the PID constants of the flywheel */
    public default PID getPID() { return new PID(0, 0, 0, 0); };
}