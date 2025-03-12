package frc.robot.subsystems.algae;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;

public class AlgaeSubsystem extends AdvancedSubsystem {
    private static AlgaeSubsystem instance;

    private AlgaePivotStates pivotState;
    private AlgaeDriveStates driveState;

    private final HashMap<AlgaePivotStates, Angle> pivotAnglesMap = new HashMap<>();
    private final HashMap<AlgaeDriveStates, Voltage> driveVoltageMap = new HashMap<>();

    private Voltage currentTargetDriveVoltage;
    private Angle currentTargetPivotAngle;

    private final AlgaeIO io;

    public AlgaeSubsystem() {
        pivotState = AlgaePivotStates.Stored;
        driveState = AlgaeDriveStates.Off;

        pivotAnglesMap.put(AlgaePivotStates.Stored, AlgaeConstants.STORED_ANGLE);
        pivotAnglesMap.put(AlgaePivotStates.Hold, AlgaeConstants.HOLD_ANGLE);
        pivotAnglesMap.put(AlgaePivotStates.Barge, AlgaeConstants.BARGE_ANGLE);
        pivotAnglesMap.put(AlgaePivotStates.Processor, AlgaeConstants.PROCESSOR_ANGLE);
        pivotAnglesMap.put(AlgaePivotStates.IntakeGround, AlgaeConstants.GROUND_ANGLE);
        pivotAnglesMap.put(AlgaePivotStates.IntakeReef, AlgaeConstants.REEF_ANGLE);
        pivotAnglesMap.put(AlgaePivotStates.IntakeCoral, AlgaeConstants.CORAL_ANGLE);

        currentTargetPivotAngle = pivotAnglesMap.get(pivotState);

        driveVoltageMap.put(AlgaeDriveStates.Off, Voltage.ofBaseUnits(0, Units.Volts));
        driveVoltageMap.put(AlgaeDriveStates.Hold, AlgaeConstants.HOLD_VOLTAGE);
        driveVoltageMap.put(AlgaeDriveStates.Intake, AlgaeConstants.INTAKE_VOLTAGE);
        driveVoltageMap.put(AlgaeDriveStates.Barge, AlgaeConstants.BARGE_VOLTAGE);
        driveVoltageMap.put(AlgaeDriveStates.Processor, AlgaeConstants.PROCESSOR_VOLTAGE);

        currentTargetDriveVoltage = driveVoltageMap.get(driveState);

        io = new AlgaeTalonFX();
        io.setDriveVoltage(new VoltageOut(0));
        io.setTargetPivot(currentTargetPivotAngle);
    }

    @Override
    public void readPeriodic() {
        Logger.recordOutput("Algae/PivotState", pivotState);
        Logger.recordOutput("Algae/DriveState", driveState);

        io.readPeriodic();

    }

    @Override
    public void writePeriodic() {
//        if (AlgaeDriveStates.Off == driveState) {
//            if (hasAlgae()) {
//                setDriveState(AlgaeDriveStates.Hold);
//                setPivotState(AlgaePivotStates.Hold);
//            }
//        }else if (AlgaeDriveStates.Hold == driveState) {
//            if (!hasAlgae()) {
//                setDriveState(AlgaeDriveStates.Off);
//                setPivotState(AlgaePivotStates.Stored);
//            }
//        }

        
        io.writePeriodic();
    }

    @Override
    public void simulatePeriodic() {
        io.simulatePeriodic();
    }

    public boolean atPivotSetpoint() {
        return io.atPivotSetpoint();
    }
    public void setPivotState(AlgaePivotStates pivotState) {
        this.pivotState = pivotState;
        currentTargetPivotAngle = pivotAnglesMap.get(this.pivotState);
        io.setTargetPivot(currentTargetPivotAngle);
    }
    public double currentPivotAngle() {
        return io.getCurrentPivot();
    }
    public AlgaePivotStates getPivotState() {
        return pivotState;
    }

    public AlgaeDriveStates getDriveState() {
        return driveState;
    }
    public void setDriveState(AlgaeDriveStates driveState) {
        this.driveState = driveState;
        currentTargetDriveVoltage = driveVoltageMap.get(this.driveState);
        VoltageOut out = new VoltageOut(currentTargetDriveVoltage);
        if (driveState == AlgaeDriveStates.Processor) {
            out.OverrideBrakeDurNeutral = true;
        }else{
            out.OverrideBrakeDurNeutral = false;
        }
        io.setDriveVoltage(out);
    }

    public boolean hasAlgae() {
        return io.hasAlgae();
    }

    public static AlgaeSubsystem getInstance() {
        if (instance == null) {
            instance = new AlgaeSubsystem();
        }
        return instance;

    }
}
