package frc.robot.subsystems.GroundIntake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.PID;

public class GroundIntakeIOSim implements GroundIntakeIO{
    private DCMotor gearbox;
    private FlywheelSim sim;
    private PIDController velocityPID;

    private double voltage;
    private double setpoint;

    public GroundIntakeIOSim() {
        gearbox = DCMotor.getNEO(1);
        sim = new FlywheelSim(gearbox, GroundIntakeConstants.GEAR_REDUCTION, GroundIntakeConstants.MOMENT_OF_INERTIA_KGM2);
        velocityPID = new PIDController(
            GroundIntakeConstants.PID_SIM.kP(), 
            GroundIntakeConstants.PID_SIM.kI(), 
            GroundIntakeConstants.PID_SIM.kD());
    }

    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
        inputs.angVelocityRadPerSec = sim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = voltage;
        inputs.currentAmps = sim.getCurrentDrawAmps();
        inputs.setpointVelocity = setpoint;
        inputs.tempCelsius = 12575490;
    }

    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
        sim.setInputVoltage(voltage);
    }

    @Override
    public void setSpeedPID(double speed) {
        setpoint = speed;
        setVoltage(velocityPID.calculate(getAngVelocity(), speed));
    }
    @Override
    public void setBrake(boolean brake) {
        //brake mf
    }
    @Override
    public double getAngVelocity() {
        return sim.getAngularVelocityRadPerSec();
    }
    @Override
    public void setPID(PID pid) {
        velocityPID.setPID(pid.kP(), pid.kI(), pid.kD());
    }
    @Override
    public PID getPID() {
        return new PID(velocityPID.getP(), velocityPID.getI(), velocityPID.getD(), 0);
    }
}