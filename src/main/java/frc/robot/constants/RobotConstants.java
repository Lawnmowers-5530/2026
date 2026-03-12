package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;
import lombok.experimental.UtilityClass;

@UtilityClass
public class RobotConstants {
    public final CANBus canivoreBus = new CANBus("canivore");
}
