package frc.robot.constants;


import lombok.experimental.UtilityClass;

@UtilityClass
public class SpindexerConstants {
    public static final String dashboardPath = "Spindexer";

    public final int
        spindexerMotorPort = 26,
        kickerMotorPort = 27;

    public double
        spindexerForwardSpeed = 3,
        spindexerFastSpeed = 10,
        spindexerReverseSpeed = -4,
        kickerForwardSpeed = 4,
        kickerFastSpeed = 10,
        kickerReverseSpeed = -4;
}
