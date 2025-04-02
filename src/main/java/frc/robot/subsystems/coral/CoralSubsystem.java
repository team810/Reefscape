package frc.robot.subsystems.coral;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;

import static edu.wpi.first.units.Units.Volts;

public class CoralSubsystem extends AdvancedSubsystem {
    private static CoralSubsystem instance;
    private CoralMotorState motorState;
    private CoralPistonState pistonState;

    private final HashMap<CoralMotorState, Voltage> motorStateMap;
    private final HashMap<CoralPistonState, DoubleSolenoid.Value> pistonStateMap;

    private final CoralIO io;

    private Voltage currentVoltageTarget;
    private DoubleSolenoid.Value currentPistonTarget;

    private CoralSubsystem() {

        motorState = CoralMotorState.Off;
        pistonState = CoralPistonState.Store;

        motorStateMap = new HashMap<>();
        motorStateMap.put(CoralMotorState.Off, Volts.of(0));
        motorStateMap.put(CoralMotorState.Source, CoralConstants.SOURCE_VOLTAGE);
        motorStateMap.put(CoralMotorState.ReefScore, CoralConstants.REEF_SCORE_VOLTAGE);
        motorStateMap.put(CoralMotorState.TroughScore, CoralConstants.TROUGH_SCORE_VOLTAGE);
        motorStateMap.put(CoralMotorState.Hold, CoralConstants.HOLD_SCORE_VOLTAGE);
        motorStateMap.put(CoralMotorState.ReefScoreMiddle,CoralConstants.REEF_SCORE_MIDDLE);

        pistonStateMap = new HashMap<>();
        pistonStateMap.put(CoralPistonState.Store, DoubleSolenoid.Value.kForward);
        pistonStateMap.put(CoralPistonState.Hold, DoubleSolenoid.Value.kForward);
        pistonStateMap.put(CoralPistonState.Source, DoubleSolenoid.Value.kForward);
        pistonStateMap.put(CoralPistonState.Reef, DoubleSolenoid.Value.kReverse);
        pistonStateMap.put(CoralPistonState.Trough, DoubleSolenoid.Value.kForward);

        io = new CoralTalonFX();
        currentPistonTarget = io.getPistonState();

    }

    @Override
    public void readPeriodic() {
        io.readPeriodic();

        Logger.recordOutput("Coral/CoralPistonState", pistonState);
        Logger.recordOutput("Coral/CoralMotorState", motorState);
    }

    @Override
    public void writePeriodic() {
        io.writePeriodic();
    }

    @Override
    public void simulatePeriodic() {

        io.simulationPeriodic();
    }

    public CoralMotorState getMotorState() {
        return motorState;
    }

    public boolean hasCoral() {
        return io.hasCoral();
    }

    public Rotation3d getAngle() {
        if (io.getPistonState() == DoubleSolenoid.Value.kForward) {
            return CoralConstants.FWD_ANGLE;
        }else{
            return CoralConstants.REV_ANGLE;
        }
    }

    public void setCoralMotorState(CoralMotorState motorState) {
        this.motorState = motorState;
        this.currentVoltageTarget = motorStateMap.get(this.motorState);
        this.io.setVoltage(currentVoltageTarget);
    }

    public CoralPistonState getCoralPistonState() {
        return pistonState;
    }

    public void setCoralPistonState(CoralPistonState pistonState) {
        this.pistonState = pistonState;
        this.currentPistonTarget = pistonStateMap.get(this.pistonState);
        this.io.setPistonState(currentPistonTarget);
    }

    public static CoralSubsystem getInstance() {
        if (instance == null) {
            instance = new CoralSubsystem();
        }
        return instance;
    }
}

