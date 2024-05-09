package frc.robot.subsystems.pivotArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.pivotArm.PivotArmConstants.PivotArmSimConstants;

import static frc.robot.subsystems.pivotArm.PivotArmConstants.PivotArmSimConstants;

import org.littletonrobotics.junction.Logger;

public class PivotArmIOSim implements PivotArmIO {
    // Classes that simulate hardware components of the robot
    private DCMotor armGearbox;

    private ProfiledPIDController pidController;
    private ArmFeedforward feedForward;

    private SingleJointedArmSim sim;

    private double setpoint = 0;

    public PivotArmIOSim() {
        armGearbox = DCMotor.getNEO(4);

        pidController = new ProfiledPIDController(PivotArmSimConstants.PIVOT_ARM_PID_SIM[0], PivotArmSimConstants.PIVOT_ARM_PID_SIM[1], PivotArmSimConstants.PIVOT_ARM_PID_SIM[2],
            new TrapezoidProfile.Constraints(2.45, 2.45));
        pidController.setTolerance(PivotArmConstants.PID_TOLERANCE);
        feedForward = new ArmFeedforward(PivotArmSimConstants.PIVOT_ARM_FF_SIM[0], PivotArmSimConstants.PIVOT_ARM_FF_SIM[1], PivotArmSimConstants.PIVOT_ARM_FF_SIM[2], PivotArmSimConstants.PIVOT_ARM_FF_SIM[3]);

        sim = new SingleJointedArmSim(
            armGearbox,
            PivotArmConstants.GEAR_REDUCTION,
            SingleJointedArmSim.estimateMOI(PivotArmConstants.LENGTH_M, PivotArmConstants.MASS_KG),
            PivotArmConstants.LENGTH_M,
            PivotArmConstants.MIN_ANGLE_RADS,
            PivotArmConstants.MAX_ANGLE_RADS,
            true,
            0
        );
    }

    @Override
    public void updateInputs(PivotArmIOInputs inputs) {
        inputs.angleRad = getAngle();
        inputs.angVelocityRadPerSec = sim.getVelocityRadPerSec();
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
        inputs.setpointAngleRad = setpoint;
    }

    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
    }

    @Override
    public double getAngle() {
        return sim.getAngleRads();
    }

    @Override
    public void goToSetpoint(double setpoint) {
        pidController.setGoal(setpoint);
        this.setpoint = setpoint;

        double pidOutput = pidController.calculate(getAngle());
        double feedForwardOutput = feedForward.calculate(getAngle(), pidController.getSetpoint().velocity);

        Logger.recordOutput("PivotArm/PIDOutput", pidOutput);
        Logger.recordOutput("PivotArm/FeedForwardOutput", feedForwardOutput);

        setVoltage(MathUtil.clamp(pidOutput + feedForwardOutput, -4, 4));
    }

    @Override
    public boolean atSetpoint() {
        return pidController.atGoal();
    }

    @Override
    public void setPID(double p, double i, double d) {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
    }

    @Override
    public void setFF(double s, double g, double v, double a) {
        feedForward = new ArmFeedforward(s, g, v, a);
    }

    @Override
    public double[] getPID() {
        return new double[] {pidController.getP(), pidController.getI(), pidController.getD()};
    }

    @Override
    public double[] getFF() {
        return new double[] {feedForward.ks, feedForward.kg, feedForward.kv, feedForward.ka};
    }

}
