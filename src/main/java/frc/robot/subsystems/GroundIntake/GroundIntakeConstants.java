package frc.robot.subsystems.GroundIntake;

import frc.robot.Constants.PID;

public class GroundIntakeConstants {
    public static final double MASS_KG = 0.15;
    public static final double RADIUS_M = 0.03;
    public static final double MOMENT_OF_INERTIA_KGM2 = 0.5 * MASS_KG * RADIUS_M * RADIUS_M;
    public static final double GEAR_REDUCTION = 12/57;
    public static final double VOLTAGE_TOLERANCE = 1;
    public static PID PID_REAL = new PID(0, 0, 0, 0);
    public static PID PID_SIM = new PID(0, 0, 0, 0);
}