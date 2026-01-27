package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import lombok.*;

import java.lang.ref.Cleaner;

public class RobotNative implements AutoCloseable {
    private static final Cleaner CLEANER = Cleaner.create();

    private final long handle;
    private final Cleaner.Cleanable handleCleanable;

    @Setter
    @Getter
    public class LauncherSubsystem implements Subsystem {
        public enum AimMode {
            NearestHub,
            LHub,
            RHub,
            Lob
        }

        @Data
        @AllArgsConstructor
        @NoArgsConstructor
        public class LauncherControlRequest {
            private AimMode aimMode;
            private boolean active;
        }

        private LauncherControlRequest currentControlRequest = new LauncherControlRequest(AimMode.NearestHub, false);

        public void setAimMode(AimMode mode) {
            this.currentControlRequest.setAimMode(mode);
        }

        public void setActive(boolean active) {
            this.currentControlRequest.setActive(active);
        }

        public AimMode getAimMode() {
            return this.currentControlRequest.getAimMode();
        }

        public boolean getActive() {
            return this.currentControlRequest.isActive();
        }

        @Override
        public void periodic() {;
            Native.submitLauncherControlRequest(RobotNative.this.handle, this.currentControlRequest);
        }
    }

    public static class InitInfo { // any otherwise needed info, cpp code reads constants.launcherconstants directly

    }

    public RobotNative(InitInfo info) {
        this.handle = Native.initialize(info);
        this.handleCleanable = CLEANER.register(this, new Destroyer(handle));
    }

    // JNI native methods
    private static class Native {
        static {
            System.load("/usr/local/frc/third-party/libRobotNative.so");
        }

        static native long initialize(InitInfo info);
        static native void destroy(long handle);
        static native void submitLauncherControlRequest(long handle, RobotNative.LauncherSubsystem.LauncherControlRequest request);
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
