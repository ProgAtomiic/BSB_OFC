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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
  public static class OperatorConstants {
    public static final int ControlePrincipal = 0;
    public static final int ControleSecundario = 1;

  }

  public final class SwerveConstants {
    public static double VelMax = 3;
    public static double ZonaMorta = 0.15;

    public static SwerveDriveKinematics SwerveKinematics = new SwerveDriveKinematics(
        new Translation2d(Units.inchesToMeters(10.8751969), Units.inchesToMeters(10.8751969)), // Front Left
        new Translation2d(Units.inchesToMeters(10.8751969), Units.inchesToMeters(-10.8751969)), // Front Right
        new Translation2d(Units.inchesToMeters(-10.8751969), Units.inchesToMeters(10.8751969)), // Back Left
        new Translation2d(Units.inchesToMeters(-10.8751969), Units.inchesToMeters(-10.8751969))); // Back Right

  }

  public static class VisaoConstants {
    public static Transform3d LimelightToCam = new Transform3d(new Translation3d(0.292, 0.0, 0),
        new Rotation3d(0, Math.toRadians(57.5), 0)); // 11.5 polegadas freente(do meio ate a frente), angulação 57.5
    public static Transform3d ArducamToCam = new Transform3d(new Translation3d(-0.26, 0.33, 0.19),
        new Rotation3d(Math.toRadians(150), Math.toRadians(-60), 0));
  }

  public static class AlinhamentoConstants {
    public static double Esquerda = -0.14; // Qual a distancia do centro da tag
    public static double Direita = 0.13;
  }

 public static int Tag1(){
  var ally = DriverStation.getAlliance(); // Pega a aliança atual

  int tag1 = 0;
  if (ally.isPresent()) {
    if (ally.get() == Alliance.Red) {
      tag1 = 1;
    } else if (ally.get() == Alliance.Blue) {
        tag1 = 12;
  }
  }

    return tag1;
 }
  public static int Tag2(){
    var ally = DriverStation.getAlliance(); // Pega a aliança atual
        int tag2 = 0;
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                tag2 = 2;
            } else if (ally.get() == Alliance.Blue) {
                tag2 = 13;
            }
        }

        return tag2;
  }

    }










