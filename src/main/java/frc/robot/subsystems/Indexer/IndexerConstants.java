package frc.robot.subsystems.Indexer;

import frc.robot.Constants.PID;

public class IndexerConstants {
    public static final double MASS_KG = 0.15;
    public static final double RADIUS_M = 0.03;
    public static final double MOMENT_OF_INERTIA_KGM2 = 0.5 * MASS_KG * RADIUS_M * RADIUS_M;
    public static final double GEAR_REDUCTION = 1;
    public static final double VOLTAGE_TOLERANCE = 1;
    public static final double INTAKE_SPEED = 0.67;
}