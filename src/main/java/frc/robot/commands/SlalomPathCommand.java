// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.TrajectoryUtils;
import frc.robot.Constants.*;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SlalomPathCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final Trajectory slalomTrajectory;
    private RamseteCommand ramseteCommand;

    public SlalomPathCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        // Assume slalomTrajectory is generated and passed to this command
        this.slalomTrajectory = TrajectoryUtils.generateSlalomTrajectory();

        // Set up the RamseteCommand
        this.ramseteCommand = new RamseteCommand(
            slalomTrajectory,
            drivetrain::getPose,
            new RamseteController(PhysicalConstants.kRamseteB, PhysicalConstants.kRamseteZeta),
            new SimpleMotorFeedforward(AutoConstants.kS,
                                       AutoConstants.kV,
                                       AutoConstants.kA),
            PhysicalConstants.kDriveKinematics,
            drivetrain::getWheelSpeeds,
            new PIDController(PhysicalConstants.kPDriveVel, 0, 0),
            new PIDController(PhysicalConstants.kPDriveVel, 0, 0),
            drivetrain::tankDriveVolts,
            drivetrain
        );
    }

    @Override
    public void initialize() {
        // Reset odometry to the starting pose of the trajectory
        drivetrain.resetOdometry(slalomTrajectory.getInitialPose());

        // Initialize the RamseteCommand
        ramseteCommand.initialize();
    }

    @Override
    public void execute() {
        // Execute the RamseteCommand
        ramseteCommand.execute();
    }

    @Override
    public boolean isFinished() {
        // Complete the command when the trajectory is finished
        return ramseteCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        // Ensure the robot stops driving at the end of the trajectory
        drivetrain.tankDriveVolts(0, 0);

        // Notify the RamseteCommand that it has ended
        ramseteCommand.end(interrupted);
    }
}
