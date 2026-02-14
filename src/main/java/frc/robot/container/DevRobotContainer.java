// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.container;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.BuildMetadata;
import frc.robot.Telemetry;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.*;

public class DevRobotContainer {

        public class Subsystems {
                public LauncherFlywheel launcherFlywheel;
        }

        Subsystems subsystems;
        DevBindings bindings;

        BuildMetadata metadata = new BuildMetadata();

        boolean _programmingDashboard = true;

        private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

        public DevRobotContainer() {
                subsystems = new Subsystems();
                subsystems.launcherFlywheel = new LauncherFlywheel(LauncherConstants.canId);
                bindings = new DevBindings(subsystems);
        }

        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return null;
        }

        public void teleopInit() {
                // Any teleop-specific initialization code can go here.
        }

        public void teleopPeriodic() {
        }

        public void teleopExit() {
                // Any teleop-specific cleanup code can go here.
        }
}
