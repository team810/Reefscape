package frc.robot.commands.auto;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Superstructure;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.coral.CoralMotorState;
import frc.robot.subsystems.coral.CoralPistonState;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import java.util.HashMap;
import java.util.Optional;

public class AutoFactory {

    public enum StartOptions {
        Right,
        Center,
        Left,
    }

    public enum ReefOptions {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L,
        None,
    }

    public enum SourceOptions {
        Left,
        Right
    }

    private final HashMap<String, Trajectory<SwerveSample>> trajectoriesBlue = new HashMap<>();
    private final HashMap<String, Trajectory<SwerveSample>> trajectoriesRed = new HashMap<>();

    private final SendableChooser<StartOptions> startOptions = new SendableChooser<>();
    private final SendableChooser<SourceOptions> sourceOptions = new SendableChooser<>();

    private SendableChooser<ReefOptions> firstScoreOptions = new SendableChooser<>();
    private SendableChooser<ReefOptions> secondScoreOptions = new SendableChooser<>();
    private SendableChooser<ReefOptions> thirdScoreOptions = new SendableChooser<>();

    private Command autoCommand = new InstantCommand(() -> System.out.println("No auto loaded"));

    public AutoFactory() {
        trajectoriesRed.clear();
        trajectoriesBlue.clear();
        
        loadTrajectory("A_LS");
        loadTrajectory("A_RS");
        loadTrajectory("B_LS");
        loadTrajectory("B_RS");

        loadTrajectory("CST_E");
        loadTrajectory("CST_F");
        loadTrajectory("CST_G");
        loadTrajectory("CST_H");
        loadTrajectory("CST_I");
        loadTrajectory("CST_J");
        loadTrajectory("C_RS");
        loadTrajectory("D_RS");
        loadTrajectory("E_RS");
        loadTrajectory("F_RS");
        loadTrajectory("G_LS");
        loadTrajectory("G_RS");
        loadTrajectory("H_LS");
        loadTrajectory("H_RS");
        loadTrajectory("I_LS");
        loadTrajectory("J_LS");
        loadTrajectory("K_LS");
        loadTrajectory("LST_G");
        loadTrajectory("LST_H");
        loadTrajectory("LST_I");
        loadTrajectory("LST_J");
        loadTrajectory("LST_K");
        loadTrajectory("LST_L");
        loadTrajectory("LS_A");
        loadTrajectory("LS_B");
        loadTrajectory("LS_G");
        loadTrajectory("LS_H");
        loadTrajectory("LS_I");
        loadTrajectory("LS_J");
        loadTrajectory("LS_K");
        loadTrajectory("LS_L");
        loadTrajectory("L_LS");
        loadTrajectory("RST_C");
        loadTrajectory("RST_D");
        loadTrajectory("RST_E");
        loadTrajectory("RST_F");
        loadTrajectory("RST_G");
        loadTrajectory("RST_H");
        loadTrajectory("RS_A");
        loadTrajectory("RS_B");
        loadTrajectory("RS_C");
        loadTrajectory("RS_D");
        loadTrajectory("RS_E");
        loadTrajectory("RS_F");
        loadTrajectory("RS_G");
        loadTrajectory("RS_H");

        startOptions.setDefaultOption("Center", StartOptions.Center);
        startOptions.addOption("Right", StartOptions.Right);
        startOptions.addOption("Left", StartOptions.Left);

        sourceOptions.setDefaultOption("Right", SourceOptions.Right);
        sourceOptions.addOption("Left", SourceOptions.Left);

        sourceOptions.onChange(event -> {updateOptions(); generateAuto();});
        startOptions.onChange(event -> {updateOptions(); generateAuto();});

        firstScoreOptions.setDefaultOption("H", ReefOptions.H);
        secondScoreOptions.setDefaultOption("H", ReefOptions.H);
        thirdScoreOptions.setDefaultOption("H", ReefOptions.H);

        SmartDashboard.putData("Starting Location", startOptions);
        SmartDashboard.putData("Source Location", sourceOptions);

        SmartDashboard.putData("First Score", firstScoreOptions);
        SmartDashboard.putData("Second Score", secondScoreOptions);
        SmartDashboard.putData("Third Score", thirdScoreOptions);

        updateOptions();
    }

    private void setLeftSecondAndThirdOptions() {

        secondScoreOptions.addOption("A", ReefOptions.A);
        secondScoreOptions.addOption("B", ReefOptions.B);
        secondScoreOptions.addOption("G", ReefOptions.G);
        secondScoreOptions.addOption("H", ReefOptions.H);
        secondScoreOptions.addOption("I", ReefOptions.I);
        secondScoreOptions.addOption("J", ReefOptions.J);
        secondScoreOptions.addOption("K", ReefOptions.K);
        secondScoreOptions.addOption("L", ReefOptions.L);

        secondScoreOptions.setDefaultOption("None", ReefOptions.None);

        thirdScoreOptions.addOption("A", ReefOptions.A);
        thirdScoreOptions.addOption("B", ReefOptions.B);
        thirdScoreOptions.addOption("G", ReefOptions.G);
        thirdScoreOptions.addOption("H", ReefOptions.H);
        thirdScoreOptions.addOption("I", ReefOptions.I);
        thirdScoreOptions.addOption("J", ReefOptions.J);
        thirdScoreOptions.addOption("K", ReefOptions.K);
        thirdScoreOptions.addOption("L", ReefOptions.L);

        thirdScoreOptions.setDefaultOption("None", ReefOptions.None);
    }

    private void setRightSecondAndThirdOptions() {
        secondScoreOptions.addOption("A", ReefOptions.A);
        secondScoreOptions.addOption("B", ReefOptions.B);
        secondScoreOptions.addOption("C", ReefOptions.C);
        secondScoreOptions.addOption("D", ReefOptions.D);
        secondScoreOptions.addOption("E", ReefOptions.E);
        secondScoreOptions.addOption("F", ReefOptions.F);
        secondScoreOptions.addOption("G", ReefOptions.G);
        secondScoreOptions.addOption("H", ReefOptions.H);

        secondScoreOptions.setDefaultOption("None", ReefOptions.None);

        thirdScoreOptions.addOption("A", ReefOptions.A);
        thirdScoreOptions.addOption("B", ReefOptions.B);
        thirdScoreOptions.addOption("C", ReefOptions.C);
        thirdScoreOptions.addOption("D", ReefOptions.D);
        thirdScoreOptions.addOption("E", ReefOptions.E);
        thirdScoreOptions.addOption("F", ReefOptions.F);
        thirdScoreOptions.addOption("G", ReefOptions.G);
        thirdScoreOptions.addOption("H", ReefOptions.H);

        thirdScoreOptions.setDefaultOption("None", ReefOptions.None);
    }

    private void updateOptions() {
        firstScoreOptions.close();
        secondScoreOptions.close();
        thirdScoreOptions.close();

        SmartDashboard.updateValues();

        StartOptions startSelection = startOptions.getSelected();
        SourceOptions sourceSelection = sourceOptions.getSelected();

        firstScoreOptions = new SendableChooser<>();
        secondScoreOptions = new SendableChooser<>();
        thirdScoreOptions = new SendableChooser<>();

        switch (startSelection) {
            case Left : {
                switch (sourceSelection) {
                    case Left: {
                        /*
                         * First Selection can be
                         * G, H, I, J, K, L
                         * Second and third selection can be
                         * L, K, J, I, H, G, B, A
                         */


                        firstScoreOptions.addOption("G", ReefOptions.G);
                        firstScoreOptions.addOption("H", ReefOptions.H);
                        firstScoreOptions.addOption("I", ReefOptions.I);
                        firstScoreOptions.addOption("J", ReefOptions.J);
                        firstScoreOptions.addOption("K", ReefOptions.K);
                        firstScoreOptions.addOption("L", ReefOptions.L);

                        setLeftSecondAndThirdOptions();
                        break;

                    }
                    case Right: {
                        /*
                        First Selection can be
                        G, H
                        Second and third selection can be
                        A, B, C, D, E, F, G, H
                         */

                        firstScoreOptions.addOption("G", ReefOptions.G);
                        firstScoreOptions.addOption("H", ReefOptions.H);

                        setRightSecondAndThirdOptions();
                        break;
                    }
                }
                break;
            }
            case Center : {
                switch (sourceSelection) {
                    case Left: {
                        /*
                         First selection can be
                         G, H, J, I,
                         Second and third selection can be
                          L, K, J, I, H, G, B, A
                         */

                        firstScoreOptions.addOption("G", ReefOptions.G);
                        firstScoreOptions.addOption("H", ReefOptions.H);
                        firstScoreOptions.addOption("I", ReefOptions.I);
                        firstScoreOptions.addOption("J", ReefOptions.J);
                        setLeftSecondAndThirdOptions();
                        break;
                    }
                    case Right: {
                        /*
                        First selection can be
                        G, H, F, E
                        Second and third selection can be
                        A, B, C, D, E, F, G, H
                         */
                        setRightSecondAndThirdOptions();

                        firstScoreOptions.addOption("E", ReefOptions.E);
                        firstScoreOptions.addOption("F", ReefOptions.F);
                        firstScoreOptions.addOption("G", ReefOptions.G);
                        firstScoreOptions.addOption("H", ReefOptions.H);
                        break;
                    }
                }
                break;
            }
            case Right : {
                switch (sourceSelection) {
                    case Left: {
                        /*
                        First Selection can be
                        G, H
                        Second and third selection can be

                         L, K, J, I, H, G, B, A
                         */

                        setLeftSecondAndThirdOptions();
                        firstScoreOptions.addOption("G", ReefOptions.G);
                        firstScoreOptions.addOption("H", ReefOptions.H);
                        break;
                    }
                    case Right: {
                        /*
                        First selection can be
                        H, G, F, E, C, D
                        Second and third selections
                        H, G, F, E, C, D, A, B
                         */

                        firstScoreOptions.addOption("C", ReefOptions.C);
                        firstScoreOptions.addOption("D", ReefOptions.D);
                        firstScoreOptions.addOption("E", ReefOptions.E);
                        firstScoreOptions.addOption("F", ReefOptions.F);
                        firstScoreOptions.addOption("G", ReefOptions.G);
                        firstScoreOptions.addOption("H", ReefOptions.H);

                        setRightSecondAndThirdOptions();
                        break;
                    }

                }
                break;
            }
        }

        SmartDashboard.putData("First Score", firstScoreOptions);
        SmartDashboard.putData("Second Score", secondScoreOptions);
        SmartDashboard.putData("Third Score", thirdScoreOptions);
        SmartDashboard.updateValues();

        firstScoreOptions.onChange(event -> {generateAuto();});
        secondScoreOptions.onChange(event -> {generateAuto();});
        thirdScoreOptions.onChange(event -> {generateAuto();});
    }

    private void generateAuto() {
        StartOptions startSelection = startOptions.getSelected();
        SourceOptions sourceSelection = sourceOptions.getSelected();

        ReefOptions reefTarget1 = firstScoreOptions.getSelected();
        ReefOptions reefTarget2 = secondScoreOptions.getSelected();
        ReefOptions reefTarget3 = thirdScoreOptions.getSelected();
        String startingLocation = "";
        switch (startSelection) {
            case Left : {
                startingLocation = "LST";
                break;
            }
            case Right : {
                startingLocation = "RST";
                break;
            }
            case Center : {
                startingLocation = "CST";
                break;
            }
        }
        String sourceLocation = "";
        switch (sourceSelection) {
            case Left: {
                sourceLocation = "LS";
                break;
            }
            case Right: {
                sourceLocation = "RS";
                break;
            }
        }
        if (reefTarget1 == null || reefTarget2 == null || reefTarget3 == null) {
            System.out.println("Error Generating Auto");
            autoCommand = new InstantCommand(() -> System.out.println("Problem with auto"));
            return;
        }

        String startPath = startingLocation + "_" + reefTarget1.toString();
        String toSource1 = reefTarget1.toString() + "_" + sourceLocation;
        String sourceTo2 = sourceLocation + "_" + reefTarget2.toString();
        String toSource2 = reefTarget2.toString() + "_" + sourceLocation;
        String sourceTo3 = sourceLocation + "_" + reefTarget3.toString();
        Trajectory<SwerveSample> part1;
        Trajectory<SwerveSample> part2;
        Trajectory<SwerveSample> part3;
        Trajectory<SwerveSample> part4;
        Trajectory<SwerveSample> part5;

        if (Superstructure.getInstance().getAlliance() == DriverStation.Alliance.Blue) {
            part1 = trajectoriesBlue.get(startPath);
            part2 = trajectoriesBlue.get(toSource1);
            part3 = trajectoriesBlue.get(sourceTo2);
            part4 = trajectoriesBlue.get(toSource2);
            part5 = trajectoriesBlue.get(sourceTo3);
        }else{
            part1 = trajectoriesRed.get(startPath);
            part2 = trajectoriesRed.get(toSource1);
            part3 = trajectoriesRed.get(sourceTo2);
            part4 = trajectoriesRed.get(toSource2);
            part5 = trajectoriesRed.get(sourceTo3);
        }


        if (part1 == null || part2 == null || part3 == null || part4 == null || part5 == null) {
            System.out.println("Error Generating Auto Null path");
            autoCommand = new InstantCommand(() -> System.out.println("Problem with auto"));
            return;
        }
        Pose2d finalPosePart1 = part1.getFinalPose(false).orElse(new Pose2d(-5,-5,new Rotation2d()));
        Pose2d finalPosePart2 = part2.getFinalPose(false).orElse(new Pose2d(-5,-5,new Rotation2d()));
        Pose2d finalPosePart3 = part3.getFinalPose(false).orElse(new Pose2d(-5,-5,new Rotation2d()));
        Pose2d finalPosePart4 = part4.getFinalPose(false).orElse(new Pose2d(-5,-5,new Rotation2d()));
        Pose2d finalPosePart5 = part5.getFinalPose(false).orElse(new Pose2d(-5,-5,new Rotation2d()));
        DrivetrainSubsystem.getInstance().resetPose(part1.getInitialPose(false).get());

        double raiseElevatorDistance = .5;
        double heightBeforeMove = 15;
        double delayBeforeShooting = 0;
        double delayBeforeGoingDown = .1;
        double actuatePistion = .1;

        autoCommand = new SequentialCommandGroup(
                new InstantCommand(() ->{
                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.AutoStore);
                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Hold);
                }),
                new ParallelCommandGroup(
                        generateFollowTrajectoryCommand(part1),
                        new SequentialCommandGroup(
                                new WaitUntilCommand( () ->
                                        (Math.abs(DrivetrainSubsystem.getInstance().getPose().getX() - finalPosePart1.getX()) < raiseElevatorDistance) &&
                                        (Math.abs(DrivetrainSubsystem.getInstance().getPose().getY() - finalPosePart1.getY()) < raiseElevatorDistance)

                                ),
                                new InstantCommand(() -> {
                                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.L4);
                                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Hold);
                                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Reef);
                                })
                        )
                ),
                new WaitUntilCommand(() -> ElevatorSubsystem.getInstance().atSetpoint()),
                new WaitCommand(delayBeforeShooting),
                new InstantCommand(() -> {
                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.ReefScore);
                }),
                new WaitCommand(delayBeforeGoingDown),
                new InstantCommand(() -> {
                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Source);
                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Source);
                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.StoreCoral);
                }),
                new WaitUntilCommand(() -> ElevatorSubsystem.getInstance().getCurrentHeight().in(Units.Inches) < heightBeforeMove),
                new ParallelCommandGroup(
                        generateFollowTrajectoryCommand(part2), // Moving from scoring location to source
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new InstantCommand(() -> {ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.Source);})
                        )
                ),
                new WaitUntilCommand(() -> CoralSubsystem.getInstance().hasCoral()),
                new InstantCommand(() -> {ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.StoreCoral);}),
                new InstantCommand(() -> {
                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.StoreCoral);
                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Hold);
                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Hold);
                }),
                new ParallelCommandGroup(
                        generateFollowTrajectoryCommand(part3),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() ->
                                        (Math.abs(DrivetrainSubsystem.getInstance().getPose().getX() - finalPosePart3.getX()) < raiseElevatorDistance) &&
                                        (Math.abs(DrivetrainSubsystem.getInstance().getPose().getY() - finalPosePart3.getY()) < raiseElevatorDistance)
                                ),
                                new InstantCommand(() -> {
                                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.L4);
                                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Reef);
                                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Hold);
                                })
                        )
                ),
                new WaitUntilCommand(() -> ElevatorSubsystem.getInstance().atSetpoint()),
                new WaitCommand(delayBeforeShooting),
                new InstantCommand(() -> {
                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.ReefScore);
                }),
                new WaitCommand(actuatePistion),
                new InstantCommand(() -> {
                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.StoreCoral);
                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Source);
                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Source);
                }),
                new WaitUntilCommand(() -> ElevatorSubsystem.getInstance().getCurrentHeight().in(Units.Inches) < heightBeforeMove),
                new ParallelCommandGroup(
                        new FollowTrajectory(part4),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new InstantCommand(() -> {ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.Source);})
                        )
                ),
                new WaitUntilCommand(() -> CoralSubsystem.getInstance().hasCoral()),
                new InstantCommand(() -> {ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.StoreCoral);}),
                new ParallelCommandGroup(
                        generateFollowTrajectoryCommand(part5),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() ->
                                        (Math.abs(DrivetrainSubsystem.getInstance().getPose().getX() - finalPosePart5.getX()) < raiseElevatorDistance) &&
                                        (Math.abs(DrivetrainSubsystem.getInstance().getPose().getY() - finalPosePart5.getY()) < raiseElevatorDistance)
                                ),
                                new InstantCommand(() ->{
                                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.L4);
                                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Reef);
                                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Hold);
                                })
                        )
                ),
                new WaitUntilCommand(() -> ElevatorSubsystem.getInstance().atSetpoint()),
                new WaitCommand(delayBeforeShooting),
                new InstantCommand(() -> {
                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.ReefScore);
                }),
                new WaitCommand(delayBeforeGoingDown),
                new InstantCommand(() -> {
                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.StoreCoral);
                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Off);
                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Store);
                })
        );

//
//        autoCommand = new SequentialCommandGroup(
//                new InstantCommand(() ->
//                {
//                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.AutoStore);
//                }),
//                new ParallelCommandGroup(
//                        generateFollowTrajectoryCommand(part1),
//                        new SequentialCommandGroup(
//                                new WaitUntilCommand(() ->
//                                    (Math.abs((DrivetrainSubsystem.getInstance().getPose().getX() - finalPosePart1.getX())) < .5) &&
//                                    (Math.abs((DrivetrainSubsystem.getInstance().getPose().getY() - finalPosePart1.getY())) < .5) &&
//                                            (Math.abs(DrivetrainSubsystem.getInstance().getPose().getRotation().getDegrees() - finalPosePart1.getRotation().getDegrees()) < 10)
//
//                                ),
//                                new InstantCommand(() ->{
//                                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.L2);
//                                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Reef);
//                                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Hold);
//                                })
//                        )
//                ),
//                scoreCommand(),
//                new ParallelCommandGroup(
//                        generateFollowTrajectoryCommand(part2)
//                ),
//                sourceIntake(),
//                new ParallelCommandGroup(
//                        generateFollowTrajectoryCommand(part3),
//                        new SequentialCommandGroup(
//                        new WaitUntilCommand(() ->
//                                (Math.abs((DrivetrainSubsystem.getInstance().getPose().getX() - finalPosePart3.getX())) < .5) &&
//                                        (Math.abs((DrivetrainSubsystem.getInstance().getPose().getY() - finalPosePart3.getY())) < .5) &&
//                                        (Math.abs(DrivetrainSubsystem.getInstance().getPose().getRotation().getDegrees() - finalPosePart3.getRotation().getDegrees()) < 10)
//
//                        ),
//                        new InstantCommand(() ->{
//                            ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.L2);
//                            CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Reef);
//                            CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Hold);
//                        })
//                ),
//                scoreCommand(),
//                new ParallelCommandGroup(
//                        generateFollowTrajectoryCommand(part4)
//                ),
//                sourceIntake(),
//                new ParallelCommandGroup(
//                        generateFollowTrajectoryCommand(part5)
//                )
//        ));

        System.out.println("Auto Generated");
    }

    public Command score() {
        return new SequentialCommandGroup(
                new WaitCommand(.5)
        );
    }


    public Command getAutoCommand() {
        return autoCommand;
    }

    public void loadTrajectory(String trajectoryName) {
        Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory(trajectoryName);
        if (trajectory.isPresent()) {
            trajectoriesBlue.put(trajectoryName, trajectory.get());
            trajectoriesRed.put(trajectoryName, trajectory.get().flipped());
            System.out.println("Trajectory loaded: " + trajectoryName);
        }else{
            System.out.println("Trajectory not found: " + trajectoryName);
        }
    }

    private Command scoreCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() ->{
                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.L2);
                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Reef);
                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Hold);
                }),
                new WaitCommand(.5),
//                new WaitUntilCommand(() ->ElevatorSubsystem.getInstance().atSetpoint()),
                new InstantCommand(() -> {
                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.ReefScore);
                }),
                new WaitCommand(.5),
                new InstantCommand(() -> {
                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.Source);
                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Source);
                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Source);
                })
        );
    }

    public Command sourceIntake()
    {
        return new SequentialCommandGroup(
                new WaitCommand(1),
                new InstantCommand(() -> {
                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.AutoStore);
                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Store);
                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Off);
                })
        );
    }

    private Command generateFollowTrajectoryCommand(Trajectory<SwerveSample> trajectory) {
        return new SequentialCommandGroup(
                new FollowTrajectory(trajectory),
                new GoToPose(trajectory.getFinalPose(false).get())
        );
    }
}
