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

    public Robot()
    {
        super(PERIOD);
        Logger.recordMetadata("ProjectName", "Reefscape");

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
            new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
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
        DrivetrainSubsystem.getInstance().setImuMode(0);
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
        DrivetrainSubsystem.getInstance().setImuMode(4); // Should use both the internal gyro and the external gyro. This is the recommended mode
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
