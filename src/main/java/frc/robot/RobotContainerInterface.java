package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public interface RobotContainerInterface {
    public Command getAutonomousCommand();
    public default void teleopInit() {};
    public default void teleopExit() {};
}
