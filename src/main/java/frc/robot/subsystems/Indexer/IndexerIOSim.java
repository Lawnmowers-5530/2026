package frc.robot.subsystems.Indexer;


import java.util.random.RandomGenerator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.simulation.MockLaserCan;

public class IndexerIOSim implements IndexerIO{

    MockLaserCan kickerFakeLaserCan;
    MockLaserCan spindexerFakeLaserCan;

    
    TalonFXSimState kickerSimState;
    TalonFXSimState spindexerSimState;

    //Do NOT touch these
    TalonFX kickerFake;
    TalonFX spindexerFake;

    public IndexerIOSim() {
        kickerFakeLaserCan = new MockLaserCan();
        spindexerFakeLaserCan = new MockLaserCan();
        kickerFake = new TalonFX(IndexerConstants.KICKER_MOTOR_PORT);
        kickerFake.getConfigurator().apply(IndexerConstants.kickeConfiguration);
        kickerSimState = kickerFake.getSimState();
        spindexerFake = new TalonFX(IndexerConstants.SPINDEXER_MOTOR_PORT);
        spindexerFake.getConfigurator().apply(IndexerConstants.spindexerConfiguration);
        spindexerSimState = spindexerFake.getSimState();

    }

    @Override
    public void setKickerTorqueCurrent(double amps) {
        kickerSimState.setSupplyVoltage(amps/40);

    }

    @Override
    public void setSpindexerTorqueCurrent(double amps) {
        spindexerSimState.setSupplyVoltage(amps/40);
    }

    @Override
    public Measurement getSpindexerLaserCANMeasurement() {
        Measurement measurement = spindexerFakeLaserCan.getMeasurement();
        measurement.distance_mm = RandomGenerator.getDefault().nextInt();
        return measurement;
    }

    @Override
    public Measurement getKickerLaserCANMeasurement() {
        Measurement measurement = kickerFakeLaserCan.getMeasurement();
        measurement.distance_mm = RandomGenerator.getDefault().nextInt();
        return measurement;
    }
    
}
