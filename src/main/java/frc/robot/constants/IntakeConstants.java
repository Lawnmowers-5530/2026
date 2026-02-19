package frc.robot.constants;

//@UtilityClass
public class IntakeConstants {
   
    //TODO find the actual values for these
    public static final int PIVOT_MOTOR_PORT = 24;
    public static final int RUN_MOTOR_PORT = 25;

    public static final double RUN_MOTOR_AMPS = 26;


    public static final double EXTENDED_ENCODER_POSITION = 0.348;
    public static final double TUCKED_ENCODER_POSITION = 0;

    public static final double PIVOT_HOLD_DOWN_AMPS = 2;

    public static final boolean sysIdMode = true; //TODO: DISABLE TO REDUCE CAN USAGE WHEN NOT SYSIDING
    public static final double sysIdRampRate = 1.0/1.0;
    public static final double sysIdStepVoltage = 4;
    public static final double sysIdTimeout = 0.3;

}
