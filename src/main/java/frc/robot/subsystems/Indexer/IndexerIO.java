package frc.robot.subsystems.Indexer;

import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

public interface IndexerIO {
    
    
    
    public void setKickerVoltage(double amps);

    public void setSpindexerVoltage(double amps);

    public Measurement getSpindexerLaserCANMeasurement();

    public Measurement getKickerLaserCANMeasurement();
}
