package frc.robot.subsystems.Shooter;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.Shooter.ShooterConstants.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {

  private final FlywheelSim leftSim =
      new FlywheelSim(DCMotor.getNeoVortex(1), FLYWHEEL_REDUCTION, MOMENT_OF_INERTIA);
  private final FlywheelSim rightSim =
      new FlywheelSim(DCMotor.getNeoVortex(1), FLYWHEEL_REDUCTION, MOMENT_OF_INERTIA);

  private final PIDController leftController =
      new PIDController(
          SimulationController.Kp(), SimulationController.Ki(), SimulationController.Kd());
  private final PIDController rightController =
      new PIDController(
          SimulationController.Kp(), SimulationController.Ki(), SimulationController.Kd());

  private SimpleMotorFeedforward leftFF =
      new SimpleMotorFeedforward(
          SimulationController.Ks(), SimulationController.Kv(), SimulationController.Ka());
  private SimpleMotorFeedforward rightFF =
      new SimpleMotorFeedforward(
          SimulationController.Ks(), SimulationController.Kv(), SimulationController.Ka());

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  private Double leftSetpointRPM = null;
  private Double rightSetpointRPM = null;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    leftSim.update(0.02);
    rightSim.update(0.02);
    
    if (leftSetpointRPM != null) {
      leftAppliedVolts =
          leftController.calculate(leftSim.getAngularVelocityRPM(), leftSetpointRPM)
              + leftFF.calculate(leftSetpointRPM);
      leftSim.setInputVoltage(MathUtil.clamp(leftAppliedVolts, -12.0, 12.0)); 
      Logger.recordOutput("Shooter Voltage Left", MathUtil.clamp(leftAppliedVolts, -12.0, 12.0));
    }

    if (rightSetpointRPM != null) {
      rightAppliedVolts =
          rightController.calculate(rightSim.getAngularVelocityRPM(), rightSetpointRPM)
              + rightFF.calculate(rightSetpointRPM);
      rightSim.setInputVoltage(MathUtil.clamp(rightAppliedVolts, -12.0, 12.0));
      Logger.recordOutput("Shooter Voltage Right", MathUtil.clamp(rightAppliedVolts, -12.0, 12.0));

    }

    inputs.lShooterPositionRotations +=
        Units.radiansToRotations(leftSim.getAngularVelocityRadPerSec() * 0.02);
    inputs.lShooterVelocityRPM = leftSim.getAngularVelocityRPM();
    inputs.lShooterAppliedVolts = leftAppliedVolts;
    inputs.lShooterOutputCurrent = leftSim.getCurrentDrawAmps();

    inputs.rShooterPositionRotations +=
        Units.radiansToRotations(rightSim.getAngularVelocityRadPerSec() * 0.02);
    inputs.rShooterVelocityRPM = rightSim.getAngularVelocityRPM();
    inputs.rShooterAppliedVolts = rightAppliedVolts;
    inputs.rShooterOutputCurrent = rightSim.getCurrentDrawAmps();
  }

  @Override
  public void setRPM(double RightRPM, double LeftRPM){
    rightSetpointRPM = RightRPM;
    leftSetpointRPM = LeftRPM;
  }

  @Override
  public void setVoltage(double RightVoltage, double LeftVoltage) {
    leftSetpointRPM = null;
    leftAppliedVolts = MathUtil.clamp(LeftVoltage, -12.0, 12.0);
    leftSim.setInputVoltage(leftAppliedVolts);
    
    rightSetpointRPM = null;
    rightAppliedVolts = MathUtil.clamp(RightVoltage, -12.0, 12.0);
    rightSim.setInputVoltage(rightAppliedVolts);
  }

  @Override
    public void setPID(double Kp, double Ki, double Kd) {
        rightController.setP(Kp);
        rightController.setI(Ki);
        rightController.setD(Kd);
        leftController.setP(Kp);
        leftController.setI(Ki);
        leftController.setD(Kd);
    }

    @Override
    public void setFF(double Ks, double Kv, double Ka) {
        rightFF = new SimpleMotorFeedforward(Ks, Kv, Ka);
        leftFF = new SimpleMotorFeedforward(Ks, Kv, Ka);
    }

  @Override
  public void stop() {
    setRPM(0.0);
  }
}