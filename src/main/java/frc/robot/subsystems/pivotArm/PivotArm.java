package frc.robot.subsystems.pivotArm;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.pivotArm.PivotArmConstants.FFConstants;
import frc.robot.subsystems.pivotArm.PivotArmConstants.PIDConstants;
import frc.robot.util.LoggedTunableNumber;

public class PivotArm extends SubsystemBase {
    private PivotArmIO io;
    private PivotArmIOInputsAutoLogged inputs;

    private LoggedTunableNumber logP, logI, logD;
    private LoggedTunableNumber logS, logG, logV, logA;

    private MechanismLigament2d armMechanism;

    public PivotArm(PivotArmIO io) {
        this.io = io;

        SmartDashboard.putData(getName(), this);

        PIDConstants pidConstants = io.getPID();
        FFConstants ffConstants = io.getFF();

        logP = new LoggedTunableNumber("PivotArm/kP", pidConstants.kP());
        logI = new LoggedTunableNumber("PivotArm/kI", pidConstants.kI());
        logD = new LoggedTunableNumber("PivotArm/kD", pidConstants.kD());

        logS = new LoggedTunableNumber("PivotArm/kS", ffConstants.kS());
        logG = new LoggedTunableNumber("PivotArm/kG", ffConstants.kG());
        logV = new LoggedTunableNumber("PivotArm/kV", ffConstants.kV());
        logA = new LoggedTunableNumber("PivotArm/kA", ffConstants.kA());

        armMechanism = getMechanism();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        
        // This updates the PID and FF constants on the IO if they are
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> io.setPID(logP.get(), logI.get(), logD.get()),
            logP, logI, logD
        );
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> io.setFF(logS.get(), logG.get(), logV.get(), logA.get()),
            logS, logG, logV, logA
        );

        armMechanism.setAngle(Units.radiansToDegrees(inputs.angleRad));

        Logger.processInputs(getName(), inputs);
    }

    public void move(double speed) {
        if(speed > 0 && io.getAngle() > PivotArmConstants.MAX_ANGLE_RADS) {
            speed = 0;
        }
        if(speed < 0 && io.getAngle() < PivotArmConstants.MIN_ANGLE_RADS) {
            speed = 0;
        }
        io.setVoltage(speed * 12);
    }

    public MechanismLigament2d getMechanism() {
        return new MechanismLigament2d(getName(), PivotArmConstants.LENGTH_M, 0, 5, new Color8Bit(Color.kAqua));
    }

    public void setMechanism(MechanismLigament2d mechanism) {
        this.armMechanism = mechanism;
    }

    /** A command that moves the arm to a specific setpoint using PID
     * @param setpointSupplier A function continuously returning the setpoint (in case it changes over time)
     * @param runForever If this is {@code true}, the command runs forever. Otherwise, it stops once the arm reaches the setpoint.
     */
    public Command pidCommand(DoubleSupplier setpointSupplier, boolean runForever) {
        return new FunctionalCommand(
            () -> {},
            () -> io.goToSetpoint(setpointSupplier.getAsDouble()),
            (interrupt) -> io.setVoltage(0),
            () -> {
                if(runForever) return false;
                return io.atSetpoint();
            },
            this
        );
    }

    /** A command that moves the arm to a specific setpoint
     * @param setpoint The desired setpoint angle, in radians
     * @param runForever If this is {@code true}, the command runs forever. Otherwise, it stops once the arm reaches the setpoint.
     */
    public Command pidCommand(double setpoint, boolean runForever) {
        return pidCommand(() -> setpoint, runForever);
    }

    /**
     * A command that moves the arm a certain speed
     * @param speedSupplier A function continuously returning the desired speed (-1 <= speed <= 1)
     */
    public Command manualCommand(DoubleSupplier speedSupplier) {
        return new FunctionalCommand(
            () -> {},
            () -> move(speedSupplier.getAsDouble()),
            (interrupt) -> {},
            () -> false,
            this
        );
    }

    /**
     * A command that moves the arm a certain speed
     * @param speed The desired speed (-1 <= speed <= 1)
     */
    public Command manualCommand(double speed) {
        return manualCommand(() -> speed);
    }
}