// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DoNothingCommand extends CommandBase {

    public DoNothingCommand() {
        // There are no specific requirements for this command
    }

    @Override
    public void initialize() {
        // Nothing to initialize
    }

    @Override
    public void execute() {
        // Nothing to execute
    }

    @Override
    public void end(boolean interrupted) {
        // Nothing to clean up or reset
    }

    @Override
    public boolean isFinished() {
        return true; // This command is immediately finished
    }
}
