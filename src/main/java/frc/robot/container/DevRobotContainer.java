// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.container;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.BuildMetadata;
import frc.robot.Telemetry;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Indexer.Indexer;

public class DevRobotContainer {

        public class Subsystems {
                public LauncherFlywheel launcherFlywheel;
                public CommandSwerveDrivetrain driveTrain;
                public Indexer indexer;
        }

        Subsystems subsystems;

        BuildMetadata metadata = new BuildMetadata();
        
        CommandXboxController controller;  

        boolean _programmingDashboard = true;

        private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

        public DevRobotContainer() {
                subsystems = new Subsystems();
                subsystems.launcherFlywheel = new LauncherFlywheel(LauncherConstants.canId);
                //subsystems.driveTrain = TunerConstants.createDrivetrain();
                subsystems.indexer = new Indexer();
                controller = new CommandXboxController(0);

                
                {
                        System.out.println("Sanity check");
                        controller.a().whileTrue(subsystems.indexer.feedStupidly());
                        controller.b().toggleOnTrue(subsystems.launcherFlywheel.setVelocityCommand(3500));
                        subsystems.launcherFlywheel.setDefaultCommand(subsystems.launcherFlywheel.setVelocityCommand(0));
                        controller.x().onTrue(subsystems.launcherFlywheel.setHoodAtBottom());
                        controller.y().onTrue(subsystems.launcherFlywheel.setHoodAtTop());
                      //  subsystems.driveTrain.setDefaultCommand(subsystems.driveTrain.driveCommand());
                        
                }
                
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
