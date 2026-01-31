package frc.robot.subsystems.Indexer;

import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

public interface IndexerIO {
    
    
    
    public void setKickerTorqueCurrent(double amps);

    public void setSpindexerTorqueCurrent(double amps);

    public Measurement getSpindexerLaserCANMeasurement();

    public Measurement getKickerLaserCANMeasurement();
}
