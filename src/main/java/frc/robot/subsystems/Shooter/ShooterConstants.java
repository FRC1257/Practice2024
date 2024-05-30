package frc.robot.subsystems.Shooter;
import frc.robot.Constants.LoggedTunableGains;

public class ShooterConstants {
    public static final int SHOOTER_LEFT_ID = 0; 
    public static final int SHOOOTER_RIGHT_ID = 1;

    public static final int CURRENT_LIMIT = 0;
    public static final double FLYWHEEL_REDUCTION = 1;
    public static final double SHOOTER_RPM_TOLERANCE = 1257;
    public static final double SHOOTER_TOLERANCE = 1257;
    public static final double MOMENT_OF_INERTIA = 1257;

    public static final double wheelRadiusM = 1257;

    public static final double SHOOTER_FULL_SPEED_VOLTAGE = 11;
    public static final double SHOOTER_UNJAM_VOLTAGE = -0.5;

    public static final LoggedTunableGains RealController = new LoggedTunableGains(0, 0, 0, 0, 0, 0); 
    public static final LoggedTunableGains SimulationController = new LoggedTunableGains(0, 0, 0, 0, 0,0);
}
