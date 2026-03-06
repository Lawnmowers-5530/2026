package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LedConstants;

public class LedManager extends SubsystemBase {
    private static LedManager instance;
    boolean initialized = false;

    private static final ControlRequest off =
            new SolidColor(0, 400).withColor(
                new RGBWColor(0,0,0,0)
            );

    private ControlRequest ledControlRequest;
    private int currentPriority = 0;

    private final CANdle candle;
    public LedManager() {
        // Initialize LED hardware here
        candle = new CANdle(LedConstants.canId); // Assuming CAN ID 0 for the CANdle
        initialized = true;
    }

    public static LedManager getInstance() {
        if (!instance.initialized) {
            instance = new LedManager();
        }
        return instance;
    }

    public void setRequest(ControlRequest request, int priority) {
        if (priority >= currentPriority) {
            ledControlRequest = request;
            currentPriority = priority;
        }
    }

    public String getHexColor(SolidColor request) {
        return String.format("#%02X%02X%02X", request.Color.Red, request.Color.Green, request.Color.Blue);
    }

    public String getHexColor(StrobeAnimation request) {
        return String.format("#%02X%02X%02X", request.Color.Red, request.Color.Green, request.Color.Blue);
    }

    double lastStrobeTime = Timer.getFPGATimestamp();
    boolean strobe = false;

    @Override
    public void periodic() {
        candle.setControl(ledControlRequest);

        if (ledControlRequest instanceof SolidColor) {
            SmartDashboard.putString("LED Color", getHexColor((SolidColor) ledControlRequest));
        } else if (ledControlRequest instanceof StrobeAnimation) {
            if (strobe) {
                SmartDashboard.putString("LED Color", getHexColor((StrobeAnimation) ledControlRequest));
                strobe = !strobe;
            } else {
                SmartDashboard.putString("LED Color", "#000000");
                strobe = !strobe;
            }
        } else {
            SmartDashboard.putString("LED Color", "#000000");
        }

        ledControlRequest = off; // Default to off if no other request is made
        currentPriority = 0;
    }


}
