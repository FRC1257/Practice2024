package frc.robot.commands.subsystems.GroundIntake;

import static frc.robot.Constants.ElectricalLayout;
import frc.robot.Constants.PID;

public class GroundIntakeIOSparkMax extends GroundIntakeIO{
    private CANSparkMax intakeMotor;
    private RelativeEncoder encoder;
    private SparkPIDController velocityPID;

    public GroundIntakeIOSparkMax() {
        intakeMotor = new CANSparkMax(ElectricalLayout.INTAKE_MOTOR, CANSparkMax.MotorType.kBrushless);

    }

    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
        inputs.
    }
    
    @Override
    public void setVelocityRad(double velocity) {

    }
}