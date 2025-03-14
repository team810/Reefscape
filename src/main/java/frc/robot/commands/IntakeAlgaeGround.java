package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeDriveStates;
import frc.robot.subsystems.algae.AlgaePivotStates;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class IntakeAlgaeGround extends Command {
    public enum TargetHeight {
        Ground,
        Coral
    }
    private final TargetHeight targetHeight;
    public IntakeAlgaeGround(TargetHeight targetHeight) {
        this.targetHeight = targetHeight;
    }

    @Override
    public void initialize() {
        if (targetHeight == TargetHeight.Ground) {
            ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.AlgaeFloor);
            AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.IntakeGround);
        }else{
            ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.AlgaeCoral);
            AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.IntakeCoral);
        }
        AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Intake);

    }

    @Override
    public void execute() {
        if (AlgaeSubsystem.getInstance().hasAlgae())
        {
            AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Hold);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (AlgaeSubsystem.getInstance().hasAlgae()) {
            AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Hold);
            AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Hold);
            ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.StoreAlgae);
        }else{
            ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.StoreAlgae);
            AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Hold);
            AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Stored);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
