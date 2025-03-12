package frc.robot;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.commands.AlgaeIntakeReef;
import frc.robot.commands.IntakeAlgaeGround;
import frc.robot.commands.SourceIntake;
import frc.robot.subsystems.algae.AlgaeDriveStates;
import frc.robot.subsystems.algae.AlgaePivotStates;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralMotorState;
import frc.robot.subsystems.coral.CoralPistonState;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class Superstructure {
    private static Superstructure instance;
    private DriverStation.Alliance alliance;

    private GamePiece currentPiece;

    private final PneumaticsControlModule pneumaticsControlModule;

    public Superstructure() {
        ChoreoAllianceFlipUtil.setYear(2025);
        alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

        IO.Initialize(IO.PrimaryDriverProfiles.Leo,IO.SecondaryDriverProfiles.KnollController);
        DrivetrainSubsystem.getInstance();
        ElevatorSubsystem.getInstance();
        CoralSubsystem.getInstance();
        AlgaeSubsystem.getInstance();

        pneumaticsControlModule = new PneumaticsControlModule();
        pneumaticsControlModule.enableCompressorDigital();

        DrivetrainSubsystem.getInstance().resetPose(new Pose2d(0, 0, new Rotation2d(0)));
        alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    }

    public void configureActions() {
        new Trigger(IO.getButtonValue(Controls.resetGyro)).onTrue(new InstantCommand(() -> DrivetrainSubsystem.getInstance().resetGyro()));

        new Trigger(IO.getButtonValue(Controls.AlgaeIntakeReefHigh)).whileTrue(new AlgaeIntakeReef(AlgaeIntakeReef.TargetHeight.High));
        new Trigger(IO.getButtonValue(Controls.AlgaeIntakeReefLow)).whileTrue(new AlgaeIntakeReef(AlgaeIntakeReef.TargetHeight.Low));


        new Trigger(IO.getButtonValue(Controls.AlgaeIntakeCoral)).whileTrue(new IntakeAlgaeGround(IntakeAlgaeGround.TargetHeight.Coral));
        new Trigger(IO.getButtonValue(Controls.AlgaeIntakeGround)).whileTrue(new IntakeAlgaeGround(IntakeAlgaeGround.TargetHeight.Ground));

        new Trigger(IO.getButtonValue(Controls.PositionL4)).onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.L4);
                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Hold);
                }),
                new WaitUntilCommand(() -> ElevatorSubsystem.getInstance().getCurrentHeight().in(Inches) > 5),
                new InstantCommand(() -> CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Reef))
        ));
        new Trigger(IO.getButtonValue(Controls.PositionL3)).onTrue(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.L3);
                            CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Hold);
                        }),
                        new WaitUntilCommand(() -> ElevatorSubsystem.getInstance().getCurrentHeight().in(Inches) > 5),
                        new InstantCommand(() -> CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Reef))
                )
        );

        new Trigger(IO.getButtonValue(Controls.PositionL2)).onTrue(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.L2);
                            CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Hold);
                        }),
                        new WaitUntilCommand(() -> ElevatorSubsystem.getInstance().getCurrentHeight().in(Inches) > 5),
                        new InstantCommand(() -> CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Reef))
                ));
        new Trigger(IO.getButtonValue(Controls.PositionTrough)).onTrue(new InstantCommand(() -> {
            ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.Trough);
            CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Trough);
        }));
        new Trigger(IO.getButtonValue(Controls.Store)).onTrue(new InstantCommand(() -> {
            ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.StoreCoral);
            CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Source);
            AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Stored);
            CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Off);

        }));

        new Trigger(IO.getButtonValue(Controls.BargePosition)).onTrue(new InstantCommand(() -> {
            ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.Barge);
            AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Barge);
            AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Hold);
        }));
        new Trigger(IO.getButtonValue(Controls.Source)).whileTrue(new SourceIntake());

        new Trigger(IO.getButtonValue(Controls.ScoreAlgae)).whileTrue(
                new StartEndCommand(
                        () -> {AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Barge);},
                        () -> {AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Off);}
                )
        );

        new Trigger(IO.getButtonValue(Controls.ProcessorPosition)).onTrue(
                new InstantCommand(() -> {
                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.Processor);
                    AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Processor);
                }
        ));

        new Trigger(IO.getButtonValue(Controls.ScoreCoral)).whileTrue(
                new StartEndCommand(
                        () -> {CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.ReefScore);},
                        () -> {CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Off);}
                )
        );
    }

    public void periodic() {
        if (CoralSubsystem.getInstance().hasCoral() && AlgaeSubsystem.getInstance().hasAlgae()) {
            currentPiece = GamePiece.Both;
        } else if (CoralSubsystem.getInstance().hasCoral() && !AlgaeSubsystem.getInstance().hasAlgae()) {
            currentPiece = GamePiece.Coral;
        } else if (!CoralSubsystem.getInstance().hasCoral() && AlgaeSubsystem.getInstance().hasAlgae()) {
            currentPiece = GamePiece.Algae;
        }else{
            currentPiece = GamePiece.None;
        }

        Logger.recordOutput("SuperStructure/CurrentPiece", currentPiece);
        Logger.recordOutput("SuperStructure/Alliance", alliance);
        Logger.recordOutput("SuperStructure/Pressure", pneumaticsControlModule.getPressure(0));

        Pose3d secondStage = new Pose3d(0,0,ElevatorSubsystem.getInstance().getCurrentHeight().in(Meters) * .33, new Rotation3d());
        Pose3d insideStage = new Pose3d(0,0, ElevatorSubsystem.getInstance().getCurrentHeight().in(Meters) * .66, new Rotation3d());

        Pose3d carriage = new Pose3d(0, .1, (ElevatorSubsystem.getInstance().getCurrentHeight().in(Meters)) + .18,new Rotation3d());
        Pose3d algae = new Pose3d(0, .2,carriage.getZ() + .17,new Rotation3d(Math.toRadians(-63) + AlgaeSubsystem.getInstance().currentPivotAngle(),0,0));
        Pose3d coral = new Pose3d(.15,.23,carriage.getZ() + .07,CoralSubsystem.getInstance().getAngle());

        Pose3d coralGP = new Pose3d(0,0,0,new Rotation3d());
        if (CoralSubsystem.getInstance().hasCoral()) {
            coralGP = coral;
        }
        Pose3d algaeGP = new Pose3d(0,0,0,new Rotation3d());
        if (AlgaeSubsystem.getInstance().hasAlgae()) {
            Pose3d pose = new Pose3d(DrivetrainSubsystem.getInstance().getPose());
            pose = pose.plus(new Transform3d(0,.2, carriage.getZ() + .17, new Rotation3d()));
            double angle = AlgaeSubsystem.getInstance().currentPivotAngle();
            double zOffset = .33 * Math.sin(angle);
            double yOffset = .33 * Math.cos(angle);
            pose = pose.plus(new Transform3d(0,yOffset,zOffset, new Rotation3d()));
            algaeGP = pose;
        }
        Logger.recordOutput("Mechanism", secondStage, insideStage, carriage, algae, coral);
        Logger.recordOutput("CoralGP", coralGP);
        Logger.recordOutput("AlgaeGP", algaeGP);

    }

    public double getCurrentPressure() {
        return pneumaticsControlModule.getPressure(0);
    }

    public GamePiece getCurrentPiece() {
        return currentPiece;
    }

    // Alliance Stuff
    public void disabledPeriodic() {
        // updating alliance selection
        DriverStation.Alliance currentAlliance = alliance;
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            alliance = DriverStation.Alliance.Blue;
        }else{
            alliance = DriverStation.Alliance.Red;
        }
        if (!DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(alliance)) {
            DrivetrainSubsystem.getInstance().switchAlliances();
        }
    }

    public DriverStation.Alliance getAlliance() {
        return alliance;
    }

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }
}
