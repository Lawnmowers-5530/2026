package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import lombok.experimental.UtilityClass;

@UtilityClass
public class RobotConstants {
    public final CANBus canivoreBus = new CANBus("canivore");
}
