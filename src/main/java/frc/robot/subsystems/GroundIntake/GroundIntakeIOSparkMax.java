package frc.robot.subsystems.GroundIntake;

import static frc.robot.Constants.NEO_CURRENT_LIMIT;
import static frc.robot.Constants.ElectricalLayout;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.PID;

public class GroundIntakeIOSparkMax implements GroundIntakeIO{
    private CANSparkMax intakeMotor;
    private RelativeEncoder encoder;

    private double desiredVolts;

    public GroundIntakeIOSparkMax() {
        intakeMotor = new CANSparkMax(ElectricalLayout.GROUND_INTAKE_ID, CANSparkMax.MotorType.kBrushless);

        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        intakeMotor.setInverted(true);

        encoder = intakeMotor.getEncoder();

        encoder.setPositionConversionFactor(2 * Math.PI * GroundIntakeConstants.GEAR_REDUCTION);
        encoder.setVelocityConversionFactor(2 * Math.PI * GroundIntakeConstants.GEAR_REDUCTION / 60);
    }

    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
        inputs.angVelocityRadPerSec = encoder.getVelocity();
        inputs.desiredVolts = desiredVolts;
        inputs.appliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.currentAmps = intakeMotor.getOutputCurrent();
        inputs.tempCelsius = intakeMotor.getMotorTemperature();
        inputs.isVoltageClose = Math.abs(inputs.appliedVolts - desiredVolts) < GroundIntakeConstants.VOLTAGE_TOLERANCE;
    }
    
    @Override
    public void set(double speed) {
        desiredVolts = speed * 12;
        intakeMotor.set(speed);
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
}