package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaePivotStates;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralMotorState;
import frc.robot.subsystems.coral.CoralPistonState;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class SourceIntake extends Command {
    public SourceIntake() {}

    @Override
    public void initialize() {
        CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Source);
        CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Source);
        ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.Source);
    }
    
    @Override
    public void end(boolean interrupted) {
        CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Off);
        CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Hold);

        ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.StoreCoral);
    }

    @Override
    public boolean isFinished() {
        return false;
//        return CoralSubsystem.getInstance().hasCoral();
    }
}
