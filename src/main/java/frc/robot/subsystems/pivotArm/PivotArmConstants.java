package frc.robot.subsystems.pivotArm;

import edu.wpi.first.math.util.Units;

public class PivotArmConstants {
    public static final double GEAR_REDUCTION = 21.0 / 35.0;

    public static final double MASS_KG = 2;
    public static final double LENGTH_M = 0.8;

    public static final double MAX_VELOCITY = 2.45;
    public static final double MAX_ACCELERATION = 2.45;

    public static final double MIN_ANGLE_RADS = 0;
    public static final double MAX_ANGLE_RADS = Units.degreesToRadians(95);
    public static final double ANGLE_OFFSET = 0;

    // PID constants
    public static record PIDConstants(double kP, double kI, double kD) {}
    public static record FFConstants(double kS, double kG, double kV, double kA) {}

    public static final PIDConstants PID = new PIDConstants(0, 0, 0);
    public static final FFConstants FF = new FFConstants(0, 0, 0, 0);
    public static final double PID_TOLERANCE = Units.degreesToRadians(1);

    public static class PivotArmSimConstants {
        // PID constants
        public static final PIDConstants PID_SIM = new PIDConstants(0, 0, 0);
        public static final FFConstants FF_SIM = new FFConstants(0, 0, 0, 0);
    }
}
