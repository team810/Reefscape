package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class GoToPose extends Command {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    private final Pose2d finalPose;

    public GoToPose(Pose2d finalPose) {
        xController = new PIDController(5,0,0);
        yController = new PIDController(5,0,0);
        thetaController = new PIDController(5,0,0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setSetpoint(.02);
        yController.setSetpoint(.02);
        thetaController.setSetpoint(Math.toRadians(.5));

        this.finalPose = finalPose;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Pose2d currentPose = DrivetrainSubsystem.getInstance().getPose();

        double xOutput = xController.calculate(currentPose.getX(),finalPose.getX());
        double yOutput = yController.calculate(currentPose.getY(), finalPose.getY());
        double theta = thetaController.calculate(currentPose.getRotation().getRadians(), finalPose.getRotation().getRadians());

        double linearVelocity = Math.sqrt((xOutput * xOutput) + (yOutput * yOutput));
        Rotation2d heading = new Rotation2d(xOutput, yOutput);

        xOutput = heading.getCos() * linearVelocity;
        yOutput = heading.getSin() * linearVelocity;

        ChassisSpeeds speeds = new ChassisSpeeds(xOutput, yOutput, theta);

        DrivetrainSubsystem.getInstance().setVelocityFOC(speeds);
        DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityFOC);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.off);
    }

}
