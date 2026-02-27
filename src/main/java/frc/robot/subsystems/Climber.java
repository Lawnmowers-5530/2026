package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private final TalonFX motor;
    VoltageOut climbControlRequest;

    public Climber(int canId) {
        this.motor = new TalonFX(canId);
        climbControlRequest = new VoltageOut(0).withEnableFOC(true);
    }

    public void runClimber(double dutyCycle) {
        double out = dutyCycle * 12.0;
        out = Math.min(Math.max(out, -12.0), 12.0);
        climbControlRequest = climbControlRequest.withOutput(out);
        motor.setControl(climbControlRequest);
    }

     public void stopClimber() {
        motor.stopMotor();
    }
}
