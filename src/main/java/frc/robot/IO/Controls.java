package frc.robot.IO;

import javax.xml.transform.Source;

public enum Controls {
    // Primary
    driveXVelocity,
    driveYVelocity,

    driveOmega,
    
    resetGyro,

    leftAlign,
    rightAlign,

    leftSource,
    rightSource,

    robotRelative,

    AlgaeIntakeReefHigh,
    AlgaeIntakeReefLow,
    AlgaeIntakeGround,
    AlgaeIntakeCoral,
    BargePosition,
    ProcessorPosition,
    ScoreAlgae,

    ScoreCoral,
    Source, // Button to intake from the source for algae
    PositionL4,
    PositionL3,
    PositionL2,
    PositionTrough,

    Store
}
