// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryUtils {

    /**
     * Generates a circular trajectory around a given center point.
     *
     * @param center The center point of the circle.
     * @param radius The radius of the circle.
     * @param clockwise Whether the circle should be generated clockwise.
     * @param config The trajectory configuration.
     * @return The generated circular trajectory.
     */
    public static Trajectory generateCircleTrajectory(Pose2d center, double radius, boolean clockwise, TrajectoryConfig config) {
        
        List<Pose2d> waypoints = new ArrayList<>();
        int pointsInCircle = 16; // More points will create a smoother circle

        for (int i = 0; i < pointsInCircle; i++) {
            double angle = 2 * Math.PI / pointsInCircle * i;
            if (clockwise) {
                angle = 2 * Math.PI - angle;
            }

            double x = center.getX() + radius * Math.cos(angle);
            double y = center.getY() + radius * Math.sin(angle);

            waypoints.add(new Pose2d(x, y, new Rotation2d(angle)));
        }

        return TrajectoryGenerator.generateTrajectory(waypoints, config);
    }


    
    public static Trajectory generateSlalomTrajectory() {
            TrajectoryConfig config = AutoConstants.getTrajectoryConfig();
    
            List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, new Rotation2d(0)),                           // Start
                new Pose2d(5, 0, Rotation2d.fromDegrees(0)),                   // First cone
                new Pose2d(10, -4, Rotation2d.fromDegrees(-90)),               // Turn after first cone
                new Pose2d(15, 0, Rotation2d.fromDegrees(0)),                  // Second cone
                new Pose2d(20, 4, Rotation2d.fromDegrees(90)),                 // Turn after second cone
                new Pose2d(25, 0, Rotation2d.fromDegrees(0)),                  // Third cone
                new Pose2d(30, -4, Rotation2d.fromDegrees(-90)),               // Turn after third cone
                new Pose2d(35, 0, Rotation2d.fromDegrees(0)),                  // Fourth cone
                new Pose2d(40, 4, Rotation2d.fromDegrees(90)),                 // Turn after fourth cone
                new Pose2d(45, 0, Rotation2d.fromDegrees(0)),                  // Fifth cone
                new Pose2d(48, 0, Rotation2d.fromDegrees(0)),                  // 3 feet beyond last cone
                // Circle around the last cone
                new Pose2d(48, 3, Rotation2d.fromDegrees(90)),                 // Quarter circle
                new Pose2d(45, 3, Rotation2d.fromDegrees(180)),                // Half circle
                new Pose2d(45, 0, Rotation2d.fromDegrees(270)),                // Three quarter circle
                new Pose2d(48, 0, Rotation2d.fromDegrees(0)),                  // Full circle
                // Return path, similar to the initial path but in reverse
                new Pose2d(40, -4, Rotation2d.fromDegrees(-90)),
                new Pose2d(35, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(30, 4, Rotation2d.fromDegrees(90)),
                new Pose2d(25, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(20, -4, Rotation2d.fromDegrees(-90)),
                new Pose2d(15, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(10, 4, Rotation2d.fromDegrees(90)),
                new Pose2d(5, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(0, 0, new Rotation2d(0))                            // Return to start
            );
    
            return TrajectoryGenerator.generateTrajectory(waypoints, config);
 }
    

}

