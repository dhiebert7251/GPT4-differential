// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.*;
import static frc.robot.Constants.*;


public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final XboxController driverController = new XboxController(OperatorConstants.kDriverControllerPort);

    SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureButtonBindings();
        configureAutoCommands();
        drivetrain.setDefaultCommand(new DriveWithJoysticks(drivetrain, driverController));

        initCamera();
    }

    private void configureButtonBindings() {
        // Example button bindings
        new JoystickButton(driverController, XboxController.Button.kA.value)
            .onTrue(new DriveToDistance(drivetrain, 10 * PhysicalConstants.kFeetToMeters));
        new JoystickButton(driverController, XboxController.Button.kB.value)
            .onTrue(new DriveToDistance(drivetrain, -10 * PhysicalConstants.kFeetToMeters));
        new JoystickButton(driverController, XboxController.Button.kX.value)
            .onTrue(new TurnToAngle(drivetrain, 90));
        new POVButton(driverController, 0)
            .onTrue(new TurnToAngle(drivetrain,0));
        new POVButton(driverController, 45)
            .onTrue(new TurnToAngle(drivetrain,45));
        new POVButton(driverController, 90)
            .onTrue(new TurnToAngle(drivetrain,90));
        new POVButton(driverController, 135)
            .onTrue(new TurnToAngle(drivetrain,135));
        new POVButton(driverController, 180)
            .onTrue(new TurnToAngle(drivetrain,180));
        new POVButton(driverController, 225)
            .onTrue(new TurnToAngle(drivetrain,225));
        new POVButton(driverController, 270)
            .onTrue(new TurnToAngle(drivetrain,270));
        new POVButton(driverController, 315)   
            .onTrue(new TurnToAngle(drivetrain,315));
        

    }

    private void configureAutoCommands() {
        autoChooser.setDefaultOption("Do Nothing", new DoNothingCommand());
        autoChooser.addOption("Drive Forward and Back", new AutoDriveForwardAndBack(drivetrain, 5 * PhysicalConstants.kInchesToMeters));
        autoChooser.addOption("Complex Auto", new ComplexAutoCommand(drivetrain));
        autoChooser.addOption("Slalom Path", new SlalomPathCommand(drivetrain));
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void updateDashboard() {
        // Update SmartDashboard with telemetry data
        SmartDashboard.putNumber("Total Distance", drivetrain.getTotalDistance());
        SmartDashboard.putNumber("Distance From Start", drivetrain.getDistanceFromStart());
        SmartDashboard.putNumber("Left Front Encoder", drivetrain.getLeftFrontEncoderCount());
        SmartDashboard.putNumber("Left Rear Encoder", drivetrain.getLeftRearEncoderCount());
        SmartDashboard.putNumber("Right Front Encoder", drivetrain.getRightFrontEncoderCount());
        SmartDashboard.putNumber("Right Rear Encoder", drivetrain.getRightRearEncoderCount());
        SmartDashboard.putNumber("Heading", drivetrain.getHeading());
    }

    private void initCamera() {
        // Start capturing from the USB webcam at /dev/video0
        CameraServer.startAutomaticCapture();


    }
}

