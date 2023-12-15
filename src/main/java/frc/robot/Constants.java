// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class PhysicalConstants {
        public static final double kGearboxReduction = 12.75;
        public static final double kWheelDiameterInches = 8.0;
        public static final double kWheelRadiusMeters = kWheelDiameterInches * 0.0254 / 2; // Convert to meters
        public static final double kTrackWidthInches = 29.75 + 2 * 3.0; // Total width including wheels
        public static final double kTrackWidthMeters = kTrackWidthInches * 0.0254; // Convert to meters
        public static final double kWheelbaseWidthInches = 20.0;
        public static final double kWheelbaseLengthInches = 24.0;
        public static final double kEncoderCountsPerRevolution = 80.0; // 20 * 4 for quadrature encoding
        public static final double kWheelCircumferenceInches = Math.PI * kWheelDiameterInches;
        public static final double kEncoderCountsPerInch = kEncoderCountsPerRevolution / (kWheelCircumferenceInches * kGearboxReduction);
        public static final double kInchesToMeters = 0.0254;
        public static final double kMetersToInches = 1 / kInchesToMeters;
        public static final double kFeetToMeters = kInchesToMeters * 12;



        // PID Constants (updated after tuning)
        public static final double kDriveP = 0.1;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kTurnP = 0.1;
        public static final double kTurnI = 0.0;
        public static final double kTurnD = 0.0;
        public static final double kPDriveVel = 1.0; // Tune this
        public static final double kIDriveVel = 0.0; // Tune this
        public static final double kDDriveVel = 0.0; // Tune this

        // CIM Motor Specifications
        public static final double kCIMMotorFreeSpeedRpm = 5310.0;
        public static final double kCIMMotorFreeCurrentAmps = 2.7;
        public static final double kCIMMotorStallCurrentAmps = 133.0;
        public static final double kCIMMotorStallTorqueInOz = 343.4;

        public static final double wheelRPM = kCIMMotorFreeSpeedRpm / kGearboxReduction;

        public static final double wheelCircumferenceMeters = kWheelDiameterInches * 0.0254 * Math.PI;
        public static final double maxSpeedMetersPerSecond = wheelRPM * wheelCircumferenceMeters / 60.0;

        // Drive Kinematics
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

        // Other Constants for Trajectory (Example values, tune as needed)
        public static final double kMaxSpeedMetersPerSecond = 1; // Example, to be tuned
        public static final double kMaxAccelerationMetersPerSecondSquared = 1; // Example, to be tuned
        public static final double kRamseteB = 2.0; // Ramsete tuning parameter
        public static final double kRamseteZeta = 0.7; // Ramsete tuning parameter


    

    }

    public static final class DriveConstants {
        public static final int kLeftFrontMotorPort = 20;
        public static final int kLeftRearMotorPort = 21;
        public static final int kRightFrontMotorPort = 22;
        public static final int kRightRearMotorPort = 23;
        public static final int[] kLeftFrontEncoderPorts = {0, 1};
        public static final int[] kLeftRearEncoderPorts = {2, 3};
        public static final int[] kRightFrontEncoderPorts = {4, 5};
        public static final int[] kRightRearEncoderPorts = {6, 7};
        public static boolean kGyroReversed = false;
    }

    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class VisionConstants {
        public static final double cameraHeightMeters = 6 * PhysicalConstants.kInchesToMeters;
        public static final double targetHeightMeters = 48 * PhysicalConstants.kInchesToMeters; //april tag center is 48 inches off the ground
        public static final double cameraPitchRadians = 0; //camera is parallel to the ground

        public static final double focalLengthPixels = 320; //320x240 resolution
        public static final double actualTargetSizeMeters = 7 * PhysicalConstants.kInchesToMeters; //7 inches 

    }

    public static final class AutoConstants {
        // ... Calculated max speed and acceleration ...

        
        // Example feedforward gains, which should be tuned for your robot
        public static final double kS = 1.0;
        public static final double kV = 1.0;
        public static final double kA = 0.1;

        // Setting up the TrajectoryConfig
        public static TrajectoryConfig getTrajectoryConfig() {
            var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(kS, kV, kA),
                PhysicalConstants.kDriveKinematics,
                10);

            return new TrajectoryConfig(PhysicalConstants.kMaxSpeedMetersPerSecond, PhysicalConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(PhysicalConstants.kDriveKinematics)
                .addConstraint(autoVoltageConstraint);
        }
    }

  }

