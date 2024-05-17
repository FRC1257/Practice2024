package frc.robot.subsystems.GroundIntake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.PID;

public class GroundIntakeIOSim implements GroundIntakeIO{
    private DCMotor gearbox;
    private FlywheelSim sim;

    private double voltage;

    public GroundIntakeIOSim() {
        gearbox = DCMotor.getNEO(1);
        sim = new FlywheelSim(gearbox, GroundIntakeConstants.GEAR_REDUCTION, GroundIntakeConstants.MOMENT_OF_INERTIA_KGM2);
    }

    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
        inputs.angVelocityRadPerSec = sim.getAngularVelocityRadPerSec();
        inputs.desiredVolts = voltage;
        inputs.appliedVolts = voltage;
        inputs.currentAmps = sim.getCurrentDrawAmps();
        inputs.tempCelsius = 12575490;
        inputs.isVoltageClose = true;
    }

    @Override
    public void set(double speed) {
        this.voltage = speed * 12;
        sim.setInputVoltage(speed * 12);
    }

    @Override
    public void setBrake(boolean brake) {
        //brake mf
    }
}