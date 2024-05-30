package frc.robot.subsystems.Indexer;

import static frc.robot.Constants.NEO_CURRENT_LIMIT;
import static frc.robot.Constants.ElectricalLayout;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.PID;

public class IndexerIOSparkMax implements IndexerIO{
    private CANSparkMax intakeMotor;
    private RelativeEncoder encoder;
    private DigitalInput breakBeam;

    private double desiredVolts;

    public IndexerIOSparkMax() {
        intakeMotor = new CANSparkMax(ElectricalLayout.INDEXER_ID, CANSparkMax.MotorType.kBrushless);

        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        intakeMotor.setInverted(true);

        encoder = intakeMotor.getEncoder();

        encoder.setPositionConversionFactor(2 * Math.PI * IndexerConstants.GEAR_REDUCTION);
        encoder.setVelocityConversionFactor(2 * Math.PI * IndexerConstants.GEAR_REDUCTION / 60);

        breakBeam = new DigitalInput(ElectricalLayout.INDEXER_BREAK_BEAM);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.angVelocityRadPerSec = encoder.getVelocity();
        inputs.desiredVolts = desiredVolts;
        inputs.appliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.currentAmps = intakeMotor.getOutputCurrent();
        inputs.tempCelsius = intakeMotor.getMotorTemperature();
        inputs.isVoltageClose = Math.abs(inputs.appliedVolts - desiredVolts) < IndexerConstants.VOLTAGE_TOLERANCE;
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

    @Override
    public boolean isIntaked() {
        return breakBeam.get();
    }
}