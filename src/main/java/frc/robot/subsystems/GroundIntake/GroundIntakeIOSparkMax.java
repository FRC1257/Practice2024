package frc.robot.subsystems.GroundIntake;

import static frc.robot.Constants.NEO_CURRENT_LIMIT;
import static frc.robot.Constants.ElectricalLayout;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.PID;

public class GroundIntakeIOSparkMax implements GroundIntakeIO{
    private CANSparkMax intakeMotor;
    private RelativeEncoder encoder;
    private SparkPIDController velocityPID;

    private double setpoint = 0;

    public GroundIntakeIOSparkMax() {
        intakeMotor = new CANSparkMax(ElectricalLayout.GROUND_INTAKE_ID, CANSparkMax.MotorType.kBrushless);

        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        intakeMotor.setInverted(true);

        encoder = intakeMotor.getEncoder();

        encoder.setPositionConversionFactor(2 * Math.PI * GroundIntakeConstants.GEAR_REDUCTION);
        encoder.setVelocityConversionFactor(2 * Math.PI * GroundIntakeConstants.GEAR_REDUCTION / 60);

        velocityPID = intakeMotor.getPIDController();

        velocityPID.setP(GroundIntakeConstants.pid.kP());
        velocityPID.setI(GroundIntakeConstants.pid.kI());
        velocityPID.setD(GroundIntakeConstants.pid.kD());
        velocityPID.setFF(GroundIntakeConstants.pid.kFF());
    }

    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
        inputs.angleRad = encoder.getPosition();
        inputs.angVelocityRadPerSec = encoder.getVelocity();
        inputs.appliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.currentAmps = new double[]{intakeMotor.getOutputCurrent()};
        inputs.tempCelsius = new double[]{intakeMotor.getMotorTemperature()};
        inputs.setpointVelocity = setpoint;
    }
    
    @Override
    public void setVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }

    @Override
    public double getAngVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void setBrake(boolean brake) {
        if (brake == true) {
            intakeMotor.setIdleMode(IdleMode.kBrake);
        } else {
            intakeMotor.setIdleMode(IdleMode.kCoast);
        }
    }

    @Override
    public void setPID(PID pid) {
        velocityPID.setP(pid.kP());
        velocityPID.setI(pid.kI());
        velocityPID.setD(pid.kD());
        velocityPID.setFF(pid.kFF());
    }

    @Override
    public PID getPID() {
        return new PID(
            velocityPID.getP(),
            velocityPID.getI(),
            velocityPID.getD(), 
            velocityPID.getFF()
        );
    }
}