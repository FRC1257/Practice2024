package frc.robot.subsystems.pivotArm;

import org.littletonrobotics.junction.AutoLog;

import static frc.robot.subsystems.pivotArm.PivotArmConstants.PIDConstants;
import static frc.robot.subsystems.pivotArm.PivotArmConstants.FFConstants;

public interface PivotArmIO {
    @AutoLog
    public static class PivotArmIOInputs {
        public double angleRad;
        public double angVelocityRadPerSec;
        public double appliedVolts;
        public double[] currentAmps;
        public double[] tempCelsius;
        public double setpointAngleRad;
    }

    /** Updates the set of logged inputs */
    public default void updateInputs(PivotArmIOInputs inputs) {}

    /** Runs open loop at the specified voltage */
    public default void setVoltage(double volts) {}

    /** Returns the current angle of the arm, in radians */
    public default double getAngle() { return 0; }

    /** Sets the desired arm voltage in order to go to the specified setpoint */
    public default void goToSetpoint(double setpoint) {}

    /** Returns {@code true} if arm is close enough to the setpoint, {@code false} otherwise */
    public default boolean atSetpoint() { return false; }

    /** Sets motor idle mode to brake if {@code brake} is {@code true}, coast if {@code false} */
    public default void setBrake(boolean brake) {}

    /** Updates PID constants */
    public default void setPID(double p, double i, double d) {}

    /** Updates FeedForward constants */
    public default void setFF(double s, double g, double v, double a) {}

    /** Returns an array containing the current PID constants */
    public default PIDConstants getPID() { return new PIDConstants(0, 0, 0); }
    
    /** Returns an array containing the current FeedForward constants */
    public default FFConstants getFF() { return new FFConstants(0, 0, 0, 0); }
}
