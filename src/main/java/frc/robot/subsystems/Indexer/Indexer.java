package frc.robot.subsystems.Indexer;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.IndexerConstants;

public class Indexer extends SubsystemBase{
    public enum States {
        Running, Idle, Empty, Jamming
    }

    IndexerIO io;

    private Debouncer spindexerJamDebouncer;
    private Debouncer kickerNotSeeingBallsDebouncer;

    private States currentState;

    private static boolean exists = false;
    public static Indexer instance;
    
   
    
   
    private Optional<Integer> lastSpindexerLaserCANMeasurement;

    public Indexer() {
        if (exists) {
            System.err.println("Malformed Code! Two instances of spindexer being created");
        }else {
            exists = true;
            instance = this;
  
             io = new IndexerIOReal();
            

            spindexerJamDebouncer = new Debouncer(IndexerConstants.SPINDEXER_JAM_TIME_THRESHOLD, DebounceType.kRising);
            kickerNotSeeingBallsDebouncer = new Debouncer(IndexerConstants.KICKER_NOT_SEEING_BALLS_TIME, DebounceType.kFalling);
            
        }
        
       
    }
    public Command feedShooter() {
        return Commands.runOnce(() -> {
            io.setSpindexerVoltage(IndexerConstants.SPINDEXER_RUN_CURRENT);
            io.setKickerVoltage(IndexerConstants.KICKER_RUN_CURRENT);
        }, this).andThen(Commands.waitUntil(this::detectJamOrEmpty))
        .andThen(Commands.either(stopIndexer(), unjam(), () -> {return currentState == States.Empty;}))
        ;
    }
    public Command stopIndexer() {
        return Commands.runOnce(() -> {
            io.setSpindexerVoltage(0);
            io.setKickerVoltage(0);
            setState(States.Idle);
        }, this);
    }
    public Command unjam() {
        return Commands.runOnce(()-> {
            io.setSpindexerVoltage(-IndexerConstants.SPINDEXER_RUN_CURRENT);

        }).andThen(Commands.waitSeconds(0.2))
        .andThen(feedShooter());
    }
    public Command feedStupidly() {
        return Commands.run(()->{
            io.setKickerVoltage(IndexerConstants.KICKER_RUN_CURRENT);
            io.setSpindexerVoltage(IndexerConstants.SPINDEXER_RUN_CURRENT);
        }, this).finallyDo(()->{
            io.setKickerVoltage(0);
            io.setSpindexerVoltage(0);
        });
    }

    public States getState() {
        return currentState;
    }
    private void setState(States state) {
        currentState = state;
    }
    private boolean detectJamOrEmpty() {
        Measurement spindexerMeasurement = io.getSpindexerLaserCANMeasurement();
            boolean sameMeasurementAsLastFrame = false;
            if (lastSpindexerLaserCANMeasurement.isPresent()) {
                if (spindexerMeasurement.is_long) {
                    lastSpindexerLaserCANMeasurement = Optional.empty();
                }else {
                    int differenceMM = Math.abs(spindexerMeasurement.distance_mm - lastSpindexerLaserCANMeasurement.get());
                    if (differenceMM <= IndexerConstants.SPINDEXER_LASERCAN_VARIANCE) {
                        sameMeasurementAsLastFrame = true;
                    }
                }

            }else {
                if (!spindexerMeasurement.is_long) {
                    lastSpindexerLaserCANMeasurement = Optional.of(spindexerMeasurement.distance_mm);
                }else {
                     sameMeasurementAsLastFrame = true;
                }
               

            }
            boolean kickerLaserCanSeesBallsPause = false;
            Measurement kickerLaserCanMeasurement = io.getKickerLaserCANMeasurement();
            if (!kickerLaserCanMeasurement.is_long) {
                kickerLaserCanSeesBallsPause = kickerLaserCanMeasurement.distance_mm <= IndexerConstants.KICKER_LASERCAN_THRESHOLD;
            }

            boolean jammingOrEmpty = spindexerJamDebouncer.calculate(sameMeasurementAsLastFrame) && kickerNotSeeingBallsDebouncer.calculate(kickerLaserCanSeesBallsPause);
            if (jammingOrEmpty) {
                if (lastSpindexerLaserCANMeasurement.isPresent()) {
                    setState(States.Empty);
                }else {
                    setState(States.Jamming);
                }
            }else {
                setState(States.Running);
            }
            return jammingOrEmpty;
    }

    @Override 
    public void periodic() {
        //TODO telemetry and logging
        SmartDashboard.putBoolean("Empty Hopper", currentState == States.Empty);
        SmartDashboard.putBoolean("Hopper Jammed", currentState == States.Jamming);
    }
}
