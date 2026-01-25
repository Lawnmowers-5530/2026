// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.BuildMetadata;

public class RobotContainer {

  BuildMetadata metadata = new BuildMetadata();

  boolean _programmingDashboard = true;
  
  public RobotContainer() {
    configureBindings();
    
    //#region Smart Dashboard Programming Related
    if (_programmingDashboard) {
      SmartDashboard.putData(metadata);
    }
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
