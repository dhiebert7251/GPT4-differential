// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.*;

public class DriveToDistance extends PIDCommand {

    public DriveToDistance(Drivetrain drivetrain, double distanceMeters) {
        super(
            new PIDController(PhysicalConstants.kDriveP, PhysicalConstants.kDriveI, PhysicalConstants.kDriveD),
            drivetrain::getTotalDistance, // Method reference for getting current distance
            distanceMeters, // Setpoint distance in meters
            output -> drivetrain.arcadeDrive(output, 0), // Use output to drive robot
            drivetrain
        );
        getController().setTolerance(0.1); // Tolerance for command completion
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
