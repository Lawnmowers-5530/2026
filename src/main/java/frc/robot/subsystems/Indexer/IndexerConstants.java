package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class IndexerConstants {
    public static int KICKER_MOTOR_PORT = 27;
    public static int SPINDEXER_MOTOR_PORT = 26;
    public static int SPINDEXER_LASERCAN_PORT = 0;
    
    public static double SPINDEXER_RUN_CURRENT = 4; //volts
    public static double KICKER_RUN_CURRENT = 8; //volts
    
    public static int KICKER_LASERCAN_PORT = 0;
    
    ////#region JAM DETECTION
    
    //The shorter you make these, the faster it response to jams but the higher the chance of a misfire
    public static double KICKER_NOT_SEEING_BALLS_TIME = 0;
    public static double SPINDEXER_JAM_TIME_THRESHOLD = 0.5;
    public static double SPINDEXER_LASERCAN_VARIANCE = 0;
    public static double KICKER_LASERCAN_THRESHOLD = 0;

    public static TalonFXConfiguration spindexerConfiguration;
    public static TalonFXConfiguration kickeConfiguration;

    static {
        spindexerConfiguration = new TalonFXConfiguration();
        kickeConfiguration = new TalonFXConfiguration();
    }
}
