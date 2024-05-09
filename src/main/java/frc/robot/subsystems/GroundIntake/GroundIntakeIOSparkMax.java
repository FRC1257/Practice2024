package frc.robot.commands.subsystems.GroundIntake;

import static frc.robot.Constants.ElectricalLayout;

public class GroundIntakeIOSparkMax extends GroundIntakeIO{
    CANSparkMax topMotor;

    public GroundIntakeIOSparkMax() {
        topMotor = new CANSparkMax(ElectricalLayout.INTAKE_MOTOR, CANSparkMax.MotorType.kBrushless);
    }

    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
        inputs.
    }
    
    @Override
    public void setVelocityRad(double velocity) {
        
    }
}