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

    private final NativeInterface nativeInterface;

    @Setter
    @Getter
    public class LauncherFlywheelSubsystem implements Subsystem {
        @Data
        @Setter
        @Getter
        @AllArgsConstructor
        @NoArgsConstructor
        public static class ControlRequest {
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
            nativeInterface.submitLauncherControlRequest(RobotNative.this.handle, RPM);
        }
    }

    @AllArgsConstructor
    public static class Constants {
        Class<?> launcherConstants;
    }

    public RobotNative(Constants constants) {
        NativeInterface iface;
        synchronized (this) {
                // Attempt to load the native library, fallback to stub if it fails
                try {
                    iface = new Native();
                } catch (UnsatisfiedLinkError e) {
                    System.err.println("Native library failed to load, using stub: " + e.getMessage());
                    iface = new NativeStub();
                }
        }
        this.nativeInterface = iface;
        this.handle = nativeInterface.initialize(constants);
        this.handleCleanable = CLEANER.register(this, new Destroyer(handle, iface));
    }

    public void startNotifier() {
        nativeInterface.startNotifier();
    }

    public void stopNotifier() {
        nativeInterface.stopNotifier();
    }

    // JNI native methods
    private interface NativeInterface {
        long initialize(Constants info);
        void destroy(long handle);
        void startNotifier();
        void stopNotifier();
        void submitLauncherControlRequest(long handle, double request);
    }

    private static class Native implements NativeInterface {
        static {
            System.loadLibrary("/usr/local/frc/third-party/lib/libRobotNative.so");
        }

        @Override
        public native long initialize(Constants info);
        @Override
        public native void destroy(long handle);
        @Override
        public native void startNotifier();
        @Override
        public native void stopNotifier();
        @Override
        public native void submitLauncherControlRequest(long handle, double request);
    }

    private static class NativeStub implements NativeInterface {
        // Stub methods, only used if load fails
        @Override
        public long initialize(Constants info) { return 0; }
        @Override
        public void destroy(long handle) {}
        @Override
        public void startNotifier() {}
        @Override
        public void stopNotifier() {}
        @Override
        public void submitLauncherControlRequest(long handle, double request) {}
    }

    // Cleanup logic
    private static class Destroyer implements Runnable {
        private final long handle;
        private final NativeInterface nativeInterface;

        Destroyer(long handle, NativeInterface nativeInterface) {
            this.handle = handle;
            this.nativeInterface = nativeInterface;
        }

        @Override
        public void run() {
            if (handle != 0) {
                nativeInterface.destroy(handle);
            }
        }
    }

    @Override
    public void close() {
        handleCleanable.clean();
    }
}
