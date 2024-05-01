package frc.robot.subsystems.Shooter;

public class ShooterConstants {
    public static final int SHOOTER_LEFT_ID = 0; 
    public static final int SHOOOTER_RIGHT_ID = 1;

    public static final int CURRENT_LIMIT = 0;
    public static final double FLYWHEEL_REDUCTION = 1;
    public static final double SHOOTER_RPM_TOLERANCE = 1257;
    public static final double SHOOTER_TOLERANCE = 1257;

    public static final SHOOTER_GAINS RealControllerLeft = new SHOOTER_GAINS(0, 0, 0, 0, 0, 0, 0); 
    public static final SHOOTER_GAINS RealControllerRight = new SHOOTER_GAINS(0, 0, 0, 0, 0, 0, 0); 
    public static final SHOOTER_GAINS SimulationController = new SHOOTER_GAINS(0, 0, 0, 0, 0, 0, 0);

    public static record SHOOTER_GAINS (double Kp, double Ki, double Kd, double Ks, double Kv, double Kg, double Ka){}
}
