package frc.robot.subsystems;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lombok.*;

import java.lang.ref.Cleaner;

public class RobotNative implements AutoCloseable {
    private static final Cleaner CLEANER = Cleaner.create();

    private final long handle;
    private final Cleaner.Cleanable handleCleanable;

    @Setter
    @Getter
    public class LauncherFlywheelSubsystem implements Subsystem {
        @Data
        @Setter
        @Getter
        @AllArgsConstructor
        @NoArgsConstructor
        public class ControlRequest {
            private double magnitude = 0;
            private AngularVelocityUnit unit = Units.Revolutions.per(Units.Minute);
        }

        private ControlRequest currentControlRequest = new ControlRequest();

        public void setControlRequest(double magnitude, AngularVelocityUnit unit) {
            this.currentControlRequest = new ControlRequest(magnitude, unit);
        }

        public LauncherFlywheelSubsystem() {}

        @Override
        public void periodic() {
            var RPM = Units.Revolutions.per(Units.Minute).convertFrom(currentControlRequest.magnitude, currentControlRequest.unit);
            Native.submitLauncherControlRequest(RobotNative.this.handle, RPM);
        }
    }

    @AllArgsConstructor
    public static class Constants {
        Class<?> launcherConstants;
    }

    public RobotNative(Constants constants) {
        this.handle = Native.initialize(constants);
        this.handleCleanable = CLEANER.register(this, new Destroyer(handle));
    }

    public void startNotifier() {
        Native.startNotifier();
    }

    public void stopNotifier() {
        Native.stopNotifier();
    }

    // JNI native methods
    private static class Native {
        static {
            System.load("/usr/local/frc/third-party/libRobotNative.so");
        }
        
        static native long initialize(Constants info);
        static native void destroy(long handle);
        static native void startNotifier();
        static native void stopNotifier();
        static native void submitLauncherControlRequest(long handle, double request);
    }

    // Cleanup logic
    private static class Destroyer implements Runnable {
        private final long handle;

        Destroyer(long handle) {
            this.handle = handle;
        }

        @Override
        public void run() {
            if (handle != 0) {
                Native.destroy(handle);
            }
        }
    }

    @Override
    public void close() {
        handleCleanable.clean();
    }
}
