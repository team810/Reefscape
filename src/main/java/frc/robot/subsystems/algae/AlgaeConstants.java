package frc.robot.subsystems.algae;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;


public class AlgaeConstants {
    public static final String CANBUS = "mech";
    public static final int PIVOT_MOTOR_ID = 7;
    public static final int DRIVE_MOTOR_ID = 4;
    public static final int LASER_ID = 6;

    public static final Angle STORED_ANGLE = Degrees.of(50);
    public static final Angle BARGE_ANGLE = Degrees.of(20);
    public static final Angle HOLD_ANGLE = Degrees.of(40);
    public static final Angle PROCESSOR_ANGLE = Degrees.of(0);
    public static final Angle GROUND_ANGLE = Degrees.of(-33.12);
    public static final Angle REEF_ANGLE = Degrees.of(-38);
    public static final Angle CORAL_ANGLE = Degrees.of(0);
    public static final Angle PIVOT_TOLERANCE = Degrees.of(.5);
    public static final Angle STARTING_ANGLE = Degrees.of(72);

    public static final Angle MAX_PIVOT = Degrees.of(55);
    public static final Angle MIN_PIVOT = Degrees.of(0);

    public static final Distance LASER_TOLERANCE = Inches.of(.5);
    public static final Distance LASER_EXPECTED = Inches.of(3.25);

    public static final Voltage INTAKE_VOLTAGE = Volts.of(-10);
    public static final Voltage BARGE_VOLTAGE = Volts.of(12);
    public static final Voltage PROCESSOR_VOLTAGE = Volts.of(5);
    public static final Voltage HOLD_VOLTAGE = Volts.of(0);


}
