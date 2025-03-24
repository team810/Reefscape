package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

public class ElevatorConstants {
    public static final int PRIMARY_MOTOR_ID = 1;
    public static final int SECONDARY_MOTOR_ID = 2;

    public static final double CONVERSION_FACTOR = 2.751; // rotations/in

    public static final Angle SOURCE_HEIGHT = Rotations.of(6.1);
    public static final Angle L4_HEIGHT = Rotations.of(27.5);
    public static final Angle L3_HEIGHT = Rotations.of(18.6);
    public static final Angle L2_HEIGHT = Rotations.of(13.05);
    public static final Angle TROUGH_HEIGHT = Rotations.of(2.8);
    public static final Angle PROCESSOR_HEIGHT = Rotations.of(0);
    public static final Angle ALGAE_HIGH_HEIGHT = Rotations.of(16.1);
    public static final Angle ALGAE_MIDDLE_HEIGHT = Rotations.of(10.4);
    public static final Angle BARGE_HEIGHT = Rotations.of(26);
    public static final Angle STORE_CORAL_HEIGHT = Rotations.of(4);
    public static final Angle STORE_ALGAE_HEIGHT = Rotations.of(4);
    public static final Angle ALGAE_FLOOR = Rotations.of(.6);
    public static final Angle ALGAE_CORAL = Rotations.of(.6);
    public static final Angle PRE_STORE_HEIGHT = Rotations.of(4.5);

    public static final Angle AUTO_STORE_HEIGHT = Rotations.of(5);

    public static final Distance DRUM_RADIUS = Inches.of(.98);
    public static final Distance ELEVATOR_MAX_HEIGHT = Inches.of(90);
}
