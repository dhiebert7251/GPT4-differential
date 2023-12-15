// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;



public class DriveWithJoysticks extends CommandBase {
    private final Drivetrain drivetrain;
    private final XboxController controller;

    //public DriveWithJoysticks(Drivetrain drivetrain) {
    public DriveWithJoysticks(Drivetrain drivetrain, XboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double forward = -controller.getLeftY(); // Forward/Backward from left joystick
        double rotation = controller.getRightX(); // Rotation from right joystick
        drivetrain.arcadeDrive(forward, rotation);
    }

    @Override
    public boolean isFinished() {
        return false; // This command should run until interrupted
    }
}


