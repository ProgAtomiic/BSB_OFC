// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public final class SwerveConstants{
    public static double VelMax = 3;
    public static double ZonaMorta = 0.15;

    public static SwerveDriveKinematics SwerveKinematics = new SwerveDriveKinematics(
      new Translation2d(Units.inchesToMeters(10.8751969), Units.inchesToMeters(10.8751969)), // Front Left
      new Translation2d(Units.inchesToMeters(10.8751969), Units.inchesToMeters(-10.8751969)), // Front Right
      new Translation2d(Units.inchesToMeters(-10.8751969), Units.inchesToMeters(10.8751969)), // Back Left
      new Translation2d(Units.inchesToMeters(-10.8751969), Units.inchesToMeters(-10.8751969)));  // Back Right
  
  }

 public static class VisaoConstants{
    public static Transform3d LimelightToCam = new Transform3d(new Translation3d(0.19, 0.0, 0.21), new Rotation3d(0, Math.toRadians(-20),0)); 
    public static Transform3d ArducamToCam = new Transform3d(new Translation3d(-0.26, 0.33, 0.19), new Rotation3d(Math.toRadians(150), Math.toRadians(-60),0));  
 }
}
