package frc.robot.subsystems.pivotArm;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotArm extends SubsystemBase {
    private PivotArmIO io;
    private PivotArmIOInputsAutoLogged inputs;

    private LoggedDashboardNumber logP, logI, logD;
    private LoggedDashboardNumber logS, logG, logV, logA;

    private MechanismLigament2d armMechanism;

    public PivotArm(PivotArmIO io) {
        this.io = io;

        SmartDashboard.putData(getName(), this);

        double[] pidConstants = io.getPID();
        double[] ffConstants = io.getFF();

        logP = new LoggedDashboardNumber("PivotArm/kP", pidConstants[0]);
        logI = new LoggedDashboardNumber("PivotArm/kI", pidConstants[1]);
        logD = new LoggedDashboardNumber("PivotArm/kD", pidConstants[2]);

        logS = new LoggedDashboardNumber("PivotArm/kS", ffConstants[0]);
        logG = new LoggedDashboardNumber("PivotArm/kG", ffConstants[1]);
        logV = new LoggedDashboardNumber("PivotArm/kV", ffConstants[2]);
        logA = new LoggedDashboardNumber("PivotArm/kA", ffConstants[3]);

        armMechanism = new MechanismLigament2d(getName(), PivotArmConstants.LENGTH_M, 0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        double[] currentPIDConstants = io.getPID();
        double[] logPIDConstants = {logP.get(), logI.get(), logD.get()};
        double[] currentFFConstants = io.getFF();
        double[] logFFConstants = {logS.get(), logG.get(), logV.get(), logA.get()};

        if(!Arrays.equals(currentPIDConstants, logPIDConstants)) {
            io.setPID(logPIDConstants[0], logPIDConstants[1], logPIDConstants[2]);
        }

        if(!Arrays.equals(currentFFConstants, logFFConstants)) {
            io.setFF(logFFConstants[0], logFFConstants[1], logFFConstants[2], logFFConstants[3]);
        }

        armMechanism.setAngle(Units.radiansToDegrees(inputs.angleRad));

        Logger.processInputs(getName(), inputs);
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
}