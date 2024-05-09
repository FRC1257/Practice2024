package frc.robot.subsystems.pivotArm;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

import static frc.robot.Constants.ElectricalLayout.PIVOT_ARM_LEFT1_ID;
import static frc.robot.Constants.ElectricalLayout.PIVOT_ARM_LEFT2_ID;
import static frc.robot.Constants.ElectricalLayout.PIVOT_ARM_RIGHT1_ID;
import static frc.robot.Constants.ElectricalLayout.PIVOT_ARM_RIGHT2_ID;
import static frc.robot.subsystems.pivotArm.PivotArmConstants.PIDConstants;
import static frc.robot.subsystems.pivotArm.PivotArmConstants.FFConstants;

import org.littletonrobotics.junction.Logger;

public class PivotArmIOSparkMax implements PivotArmIO {
    // Classes to interact with the hardware
    private CANSparkMax leftMotor1, leftMotor2, rightMotor1, rightMotor2;
    private final ProfiledPIDController pidController;
    private ArmFeedforward feedForward;

    private final SparkAbsoluteEncoder encoder;

    private double setpoint = 0;

    public PivotArmIOSparkMax() {
        leftMotor1 = new CANSparkMax(PIVOT_ARM_LEFT1_ID, MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(PIVOT_ARM_LEFT2_ID, MotorType.kBrushless);
        rightMotor1 = new CANSparkMax(PIVOT_ARM_RIGHT1_ID, MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(PIVOT_ARM_RIGHT2_ID, MotorType.kBrushless);

        leftMotor2.follow(leftMotor1, false);
        rightMotor1.follow(leftMotor1, true);
        rightMotor2.follow(leftMotor1, true);

        setBrake(true);
        
        leftMotor1.enableVoltageCompensation(12.0);

        leftMotor1.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        leftMotor2.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        rightMotor1.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        rightMotor2.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);

        leftMotor1.burnFlash();
        leftMotor2.burnFlash();
        rightMotor1.burnFlash();
        rightMotor2.burnFlash();

        // Set up Encoders
        encoder = leftMotor1.getAbsoluteEncoder(Type.kDutyCycle);
        encoder.setPositionConversionFactor(2 * Constants.PI * PivotArmConstants.GEAR_REDUCTION);
        encoder.setVelocityConversionFactor(2 * Constants.PI * PivotArmConstants.GEAR_REDUCTION / 60);

        // Set up PID
        pidController = new ProfiledPIDController(PivotArmConstants.PID.kP(), PivotArmConstants.PID.kI(), PivotArmConstants.PID.kD(), 
            new TrapezoidProfile.Constraints(2.45, 2.45));
        pidController.setTolerance(PivotArmConstants.PID_TOLERANCE);

        feedForward = new ArmFeedforward(PivotArmConstants.FF.kS(), PivotArmConstants.FF.kG(), PivotArmConstants.FF.kV(), PivotArmConstants.FF.kA());
    }

    @Override
    public void updateInputs(PivotArmIOInputs inputs) {
        inputs.angleRad = getAngle();
        inputs.angVelocityRadPerSec = encoder.getVelocity();
        inputs.appliedVolts = leftMotor1.getAppliedOutput() * leftMotor1.getBusVoltage();
        inputs.currentAmps = new double[] {leftMotor1.getOutputCurrent()};
        inputs.tempCelsius = new double[] {leftMotor1.getMotorTemperature()};
        inputs.setpointAngleRad = setpoint;
    }

    @Override
    public void setVoltage(double volts) {
        leftMotor1.setVoltage(volts);
    }

    @Override
    public double getAngle() {
        return encoder.getPosition() + PivotArmConstants.ANGLE_OFFSET;
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
    public void setBrake(boolean brake) {
        leftMotor1.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        leftMotor2.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        rightMotor1.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        rightMotor2.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
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
    public PIDConstants getPID() {
        return new PIDConstants(pidController.getP(), pidController.getI(), pidController.getD());
    }

    @Override
    public FFConstants getFF() {
        return new FFConstants(feedForward.ks, feedForward.kg, feedForward.kv, feedForward.ka);
    }
}
