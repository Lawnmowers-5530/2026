package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import frc.robot.constants.IndexerConstants;

public class IndexerIOReal implements IndexerIO {

    TalonFX spindexerMotor;
    TalonFX kickerMotor;
    LaserCan spindexerLaserCAN;
    LaserCan kickerLaserCAN;

    public IndexerIOReal() {
        spindexerMotor = new TalonFX(IndexerConstants.SPINDEXER_MOTOR_PORT, "canivore");
       // spindexerLaserCAN = new LaserCan(IndexerConstants.SPINDEXER_LASERCAN_PORT);
      //  kickerLaserCAN = new LaserCan(IndexerConstants.KICKER_LASERCAN_PORT);
         kickerMotor = new TalonFX(IndexerConstants.KICKER_MOTOR_PORT, "canivore");

       // spindexerMotor.getConfigurator().apply(IndexerConstants.spindexerConfiguration);
        //kickerMotor.getConfigurator().apply(IndexerConstants.kickeConfiguration);
    }

    @Override
    public void setKickerVoltage(double amps) {
        kickerMotor.setControl(new VoltageOut(amps).withEnableFOC(true));
    }

    @Override
    public void setSpindexerVoltage(double amps) {
        spindexerMotor.setControl(new VoltageOut(amps).withEnableFOC(true));
    }

    @Override
    public Measurement getSpindexerLaserCANMeasurement() {
        return spindexerLaserCAN.getMeasurement();
    }

    @Override
    public Measurement getKickerLaserCANMeasurement() {
        return kickerLaserCAN.getMeasurement();
    }
    
}
