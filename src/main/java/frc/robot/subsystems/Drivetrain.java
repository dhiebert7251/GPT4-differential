// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.*;


public class Drivetrain extends SubsystemBase {
    // Motor Controllers
    private final WPI_TalonSRX leftFrontMotor = new WPI_TalonSRX(DriveConstants.kLeftFrontMotorPort);
    private final WPI_VictorSPX leftRearMotor = new WPI_VictorSPX(DriveConstants.kLeftRearMotorPort);
    private final WPI_TalonSRX rightFrontMotor = new WPI_TalonSRX(DriveConstants.kRightFrontMotorPort);
    private final WPI_VictorSPX rightRearMotor = new WPI_VictorSPX(DriveConstants.kRightRearMotorPort);

    // Encoders
    private final Encoder leftFrontEncoder = new Encoder(DriveConstants.kLeftFrontEncoderPorts[0], DriveConstants.kLeftFrontEncoderPorts[1]);
    private final Encoder leftRearEncoder = new Encoder(DriveConstants.kLeftRearEncoderPorts[0], DriveConstants.kLeftRearEncoderPorts[1]);
    private final Encoder rightFrontEncoder = new Encoder(DriveConstants.kRightFrontEncoderPorts[0], DriveConstants.kRightFrontEncoderPorts[1]);
    private final Encoder rightRearEncoder = new Encoder(DriveConstants.kRightRearEncoderPorts[0], DriveConstants.kRightRearEncoderPorts[1]);

    // Gyro
    private final AHRS gyro;
    
    // Differential Drive
    private final DifferentialDrive drive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry odometry;

   private final Pose2d initialPose = new Pose2d(0, 0, new Rotation2d(0));




    public Drivetrain() {
        // Motor initialization
        leftRearMotor.follow(leftFrontMotor);
        rightRearMotor.follow(rightFrontMotor);
        rightFrontMotor.setInverted(true);
        rightRearMotor.setInverted(true);

        // Encoder configuration
        configureEncoders();

        //gyro
        gyro = new AHRS();

        new Thread(() -> {
          try {
            Thread.sleep(1000);
            zeroHeading();
          } catch (Exception e){
          }
        }).start();

        // Reset encoders and odometry
        resetEncoders();
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()),getLeftDistance(), getRightDistance(),getPose());
    }

    private void configureEncoders() {
        double distancePerPulse = (Math.PI * PhysicalConstants.kWheelDiameterInches * PhysicalConstants.kInchesToMeters) / (PhysicalConstants.kGearboxReduction * PhysicalConstants.kEncoderCountsPerRevolution);
        leftFrontEncoder.setDistancePerPulse(distancePerPulse);
        leftRearEncoder.setDistancePerPulse(distancePerPulse);
        rightFrontEncoder.setDistancePerPulse(distancePerPulse);
        rightRearEncoder.setDistancePerPulse(distancePerPulse);
    }

    public void arcadeDrive(double fwd, double rot) {
        drive.arcadeDrive(fwd, rot);
    }

    public void resetEncoders() {
        leftFrontEncoder.reset();
        leftRearEncoder.reset();
        rightFrontEncoder.reset();
        rightRearEncoder.reset();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(Rotation2d.fromDegrees(getHeading()),getLeftDistance(),getRightDistance(),pose);

    }



    @Override
    public void periodic() {
        sendTelemetry();
        
        // Update the odometry in the periodic block
        odometry.update(Rotation2d.fromDegrees(getHeading()), 
                        leftFrontEncoder.getDistance(),
                        rightFrontEncoder.getDistance());
    }

 // Additional methods

    public double getLeftFrontEncoderCount() {
        return leftFrontEncoder.get();
    }

    public double getLeftRearEncoderCount() {
        return leftRearEncoder.get();
    }

    public double getRightFrontEncoderCount() {
        return rightFrontEncoder.get();
    }

    public double getRightRearEncoderCount() {
        return rightRearEncoder.get();
    }

    public double getLeftDistance(){
      return (leftFrontEncoder.getDistance() + leftRearEncoder.getDistance()) / 2.0;
    }

    public double getRightDistance(){
      return (rightFrontEncoder.getDistance() + rightRearEncoder.getDistance()) / 2.0;
    }

    public double getTotalDistance() {
        double leftDistance = (leftFrontEncoder.getDistance() + leftRearEncoder.getDistance()) / 2.0;
        double rightDistance = (rightFrontEncoder.getDistance() + rightRearEncoder.getDistance()) / 2.0;
        return (leftDistance + rightDistance) / 2.0;
    }

   public double getDistanceFromStart() {
        Pose2d currentPose = odometry.getPoseMeters();
        return Math.sqrt(Math.pow(currentPose.getX() - initialPose.getX(), 2) 
                         + Math.pow(currentPose.getY() - initialPose.getY(), 2));
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            leftFrontEncoder.getRate(),   // Speed of the left wheel in meters per second
            rightFrontEncoder.getRate()   // Speed of the right wheel in meters per second
        );
    }

    // Additional methods for retrieving individual encoder counts, total distance, etc.
    public void zeroHeading(){
      gyro.reset();
    }

    public double getHeading() {
      return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftFrontMotor.set(ControlMode.PercentOutput, leftVolts / 12.0);
        rightFrontMotor.set(ControlMode.PercentOutput, rightVolts / 12.0);
    }

    public void sendTelemetry() {
        ShuffleboardTab tab = Shuffleboard.getTab("Telemetry");

        tab.add("Left Front Encoder", getLeftFrontEncoderCount());
        tab.add("Left Rear Encoder", getLeftRearEncoderCount());
        tab.add("Right Front Encoder", getRightFrontEncoderCount());
        tab.add("Right Rear Encoder", getRightRearEncoderCount());
        tab.add("Total Distance", getTotalDistance());
        tab.add("Distance From Start", getDistanceFromStart());
        tab.add("Heading", getHeading());
    }
}
