package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.auto.AutoFactory;
import frc.robot.lib.LimelightHelpers;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    public static final double PERIOD = .020;
    private final AutoFactory autoFactory;

    private final PowerDistribution pdp = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

    public Robot()
    {
        super(PERIOD);
        Logger.recordMetadata("ProjectName", "Reefscape");

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start();

        DriverStation.silenceJoystickConnectionWarning(true);
        CommandScheduler.getInstance().setPeriod(.015);

        Superstructure.getInstance();

        CommandScheduler.getInstance().unregisterSubsystem(DrivetrainSubsystem.getInstance());
        autoFactory = new AutoFactory();

        System.out.println("A:" + FieldConstants.BlueReef.REEF_A);
        System.out.println("B:" + FieldConstants.BlueReef.REEF_B);
        System.out.println("C:" + FieldConstants.BlueReef.REEF_C);
        System.out.println("D:" + FieldConstants.BlueReef.REEF_D);
        System.out.println("E:" + FieldConstants.BlueReef.REEF_E);
        System.out.println("F:" + FieldConstants.BlueReef.REEF_F);
        System.out.println("G:" + FieldConstants.BlueReef.REEF_G);
        System.out.println("H:" + FieldConstants.BlueReef.REEF_H);
        System.out.println("I:" + FieldConstants.BlueReef.REEF_I);
        System.out.println("J:" + FieldConstants.BlueReef.REEF_J);
        System.out.println("K:" + FieldConstants.BlueReef.REEF_K);
        System.out.println("L:" + FieldConstants.BlueReef.REEF_L);
        System.out.println("RS" + FieldConstants.BlueReef.RIGHT_SOURCE);
        System.out.println("RS" + FieldConstants.BlueReef.LEFT_SOURCE);
    }

    @Override
    public void robotInit() {
        Superstructure.getInstance().configureActions();
    }

    @Override
    public void robotPeriodic() {
        readPeriodic();
        Superstructure.getInstance().periodic();
        CommandScheduler.getInstance().run();
        writePeriodic();

        Logger.recordOutput("PDP", pdp.getTotalCurrent());
    }

    public void readPeriodic() {
        DrivetrainSubsystem.getInstance().readPeriodic();
        ElevatorSubsystem.getInstance().readPeriodic();
        CoralSubsystem.getInstance().readPeriodic();
        AlgaeSubsystem.getInstance().readPeriodic();
    }

    public void writePeriodic() {
        DrivetrainSubsystem.getInstance().writePeriodic();
        ElevatorSubsystem.getInstance().writePeriodic();
        CoralSubsystem.getInstance().writePeriodic();
        AlgaeSubsystem.getInstance().writePeriodic();
    }
    
    @Override
    public void autonomousInit() {
        DrivetrainSubsystem.getInstance().setImuMode(2);
        NetworkTableInstance.getDefault().getTable(DrivetrainConstants.LIME_LIGHT_CORAL).getEntry("throttle_set").setInteger(0);
        NetworkTableInstance.getDefault().getTable(DrivetrainConstants.LIME_LIGHT_SOURCE).getEntry("throttle_set").setInteger(0);
        CommandScheduler.getInstance().schedule(autoFactory.getAutoCommand());
    }
    
    
    @Override
    public void autonomousPeriodic() {
    }


    @Override
    public void teleopInit() {
        NetworkTableInstance.getDefault().getTable(DrivetrainConstants.LIME_LIGHT_CORAL).getEntry("throttle_set").setInteger(0);
        NetworkTableInstance.getDefault().getTable(DrivetrainConstants.LIME_LIGHT_SOURCE).getEntry("throttle_set").setInteger(0);
        DrivetrainSubsystem.getInstance().setImuMode(2); // Should use both the internal gyro and the external gyro. This is the recommended mode
        CommandScheduler.getInstance().schedule(new ManualDriveCommand());
    }

    @Override
    public void teleopPeriodic() {}
    
    
    @Override
    public void disabledInit() {
        DrivetrainSubsystem.getInstance().setImuMode(1);
        NetworkTableInstance.getDefault().getTable(DrivetrainConstants.LIME_LIGHT_CORAL).getEntry("throttle_set").setInteger(150);
        NetworkTableInstance.getDefault().getTable(DrivetrainConstants.LIME_LIGHT_SOURCE).getEntry("throttle_set").setInteger(150);
    }
    
    
    @Override
    public void disabledPeriodic() {
        Superstructure.getInstance().disabledPeriodic();
        DrivetrainSubsystem.getInstance().resetLLGyro();
    }

    @Override
    public void simulationPeriodic() {
        DrivetrainSubsystem.getInstance().simulationPeriodic();
        ElevatorSubsystem.getInstance().simulatePeriodic();
        CoralSubsystem.getInstance().simulatePeriodic();
        AlgaeSubsystem.getInstance().simulatePeriodic();
    }
}
