// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.*;

public class ComplexAutoCommand extends SequentialCommandGroup {

    public ComplexAutoCommand(Drivetrain drivetrain) {
        double distanceMeters = 5 * PhysicalConstants.kFeetToMeters; // 5 feet in meters

        addCommands(
            // Drive forward 5 feet
            new DriveToDistance(drivetrain, distanceMeters),
            
            // Rotate 90 degrees counterclockwise
            new TurnToAngle(drivetrain, -90),

            // Drive forward 5 feet
            new DriveToDistance(drivetrain, distanceMeters),

            // Rotate 90 degrees clockwise
            new TurnToAngle(drivetrain, 90),

            // Drive backward 5 feet
            new DriveToDistance(drivetrain, -distanceMeters),

            // Rotate 90 degrees clockwise
            new TurnToAngle(drivetrain, 90),

            // Drive forward 5 feet
            new DriveToDistance(drivetrain, distanceMeters),

            // Rotate 270 degrees clockwise
            new TurnToAngle(drivetrain, 270)
        );
    }
}
