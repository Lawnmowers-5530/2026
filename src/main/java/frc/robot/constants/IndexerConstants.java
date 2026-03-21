package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import lombok.experimental.UtilityClass;

@UtilityClass
public class IndexerConstants {
    public final int kickerMotorPort = 27;
    public final int spindexerMotorPort = 26;
    public final int spindexerLaserCanPort = 0;

    public final double spindexerRunCurrent = 4; //volts
    public final double kickerRunCurrent = 8; //volts

    public final int kickerLaserCanPort = 0;

    public final double jitterTime = 0.3; //seconds

    ////#region JAM DETECTION
    
    //The shorter you make these, the faster it response to jams but the higher the chance of a misfire
    public final double kickerNotSeeingBallsTime = 0;
    public final double spindexerJamTimeThreshold = 0.5;
    public final double spindexerLaserCanVariance = 0;
    public final double kickerLaserCanThreshold = 0;

    public final TalonFXConfiguration spindexerConfiguration;
    public final TalonFXConfiguration kickeConfiguration;

    static {
        spindexerConfiguration = new TalonFXConfiguration();
        kickeConfiguration = new TalonFXConfiguration();
    }
}
