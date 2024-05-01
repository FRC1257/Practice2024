package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ShooterIOSparkMax implements ShooterIO  {

    private CANSparkFlex lMotor;
    private CANSparkFlex rMotor;

    private RelativeEncoder lEncoder;
    private RelativeEncoder rEncoder; 

    private SparkPIDController lController;
    private SparkPIDController rController;

    private SimpleMotorFeedforward lFeed = new SimpleMotorFeedforward(0,0,0);
    private SimpleMotorFeedforward rFeed = new SimpleMotorFeedforward(0,0,0);

    private ShooterIOInputs inputs;

    public ShooterIOSparkMax() {
        lMotor = new CANSparkFlex(ShooterConstants.SHOOTER_LEFT_ID, CANSparkFlex.MotorType.kBrushless);
        rMotor = new CANSparkFlex(ShooterConstants.SHOOOTER_RIGHT_ID, CANSparkFlex.MotorType.kBrushless);

        lEncoder = lMotor.getEncoder();
        rEncoder = rMotor.getEncoder();

        lMotor.restoreFactoryDefaults();
        rMotor.restoreFactoryDefaults();
    
        setLeftBreak(false);
        setRightBreak(false);
    
        rMotor.setInverted(true);
        
        lMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        rMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);

        lEncoder.setPosition(0);
        rEncoder.setPosition(0);

        lEncoder.setPositionConversionFactor(1.0 / ShooterConstants.FLYWHEEL_REDUCTION); 
        rEncoder.setPositionConversionFactor(1.0 / ShooterConstants.FLYWHEEL_REDUCTION);
        lEncoder.setVelocityConversionFactor(1.0 / ShooterConstants.FLYWHEEL_REDUCTION);
        rEncoder.setVelocityConversionFactor(1.0 / ShooterConstants.FLYWHEEL_REDUCTION);

        lController = lMotor.getPIDController();
        rController = rMotor.getPIDController();
        lController.setFeedbackDevice(lEncoder);
        rController.setFeedbackDevice(rEncoder);

        lMotor.burnFlash();
        rMotor.burnFlash();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
      inputs.lShooterPositionRotations = lEncoder.getPosition();
      inputs.lShooterVelocityRPM = lEncoder.getVelocity();
      inputs.lShooterAppliedVolts = lMotor.getAppliedOutput() * 12;
      inputs.lShooterOutputCurrent = lMotor.getOutputCurrent();
  
      inputs.rShooterPositionRotations = rEncoder.getPosition();
      inputs.rShooterVelocityRPM = rEncoder.getVelocity();
      inputs.rShooterAppliedVolts = rMotor.getAppliedOutput() * 12;
      inputs.rShooterOutputCurrent = rMotor.getOutputCurrent();
    }

    @Override
    public void setRPM(double rpm) {
        lController.setReference(
            rpm,
            CANSparkBase.ControlType.kVelocity,
            0,
            rFeed.calculate(rpm),
            SparkPIDController.ArbFFUnits.kVoltage);

        rController.setReference(
            rpm,
            CANSparkBase.ControlType.kVelocity,
            0,
            rFeed.calculate(rpm),
            SparkPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void setVoltage(double volts){
        lMotor.setVoltage(volts);
        rMotor.setVoltage(volts);
    }


    @Override
    public void setLeftBreak(boolean Isenabled) {
        lMotor.setIdleMode(Isenabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }

    @Override
    public void setRightBreak(boolean Isenabled) {
        rMotor.setIdleMode(Isenabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }

    @Override
    public void setPID(double Kp, double Ki, double Kd) {
        rController.setP(Kp);
        rController.setI(Ki);
        rController.setD(Kd);
        lController.setP(Kp);
        lController.setI(Ki);
        lController.setD(Kd);
    }

    @Override
    public void setFF(double Ks, double Kv, double Ka) {
        rFeed = new SimpleMotorFeedforward(Ks, Kv, Ka);
        lFeed = new SimpleMotorFeedforward(Ks, Kv, Ka);
    }

    @Override
    public void stop() {
        setRPM(0.0);
    }
}

