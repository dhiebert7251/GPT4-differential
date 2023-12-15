// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;


public class AutoDriveForwardAndBack extends SequentialCommandGroup {

    public AutoDriveForwardAndBack(Drivetrain drivetrain, double distanceMeters) {
        addCommands(
            // Drive forward the specified distance
            new DriveToDistance(drivetrain, distanceMeters),
            
            // Wait for 3 seconds
            new WaitCommand(3),

            // Drive backward the same distance
            new DriveToDistance(drivetrain, -distanceMeters)
        );
    }
}

