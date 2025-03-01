// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.teleop.BallIntake;
import frc.robot.commands.teleop.BallShooter;
import frc.robot.subsystems.AlgaSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private static final XboxController Controle_0 = new XboxController(OperatorConstants.ControlePrincipal);
  private static final XboxController Controle_2 = new XboxController(OperatorConstants.ControleSecundario);

  public static final AlgaSubsystem m_intake_alga = new AlgaSubsystem();
  public static final BallIntake m_ball_intake = new BallIntake(m_intake_alga,Controle_0);
  public static final BallShooter m_ball_shooter = new BallShooter(m_intake_alga, Controle_0);
  ////////////////////////////////////////////////////////////////////////////////////////////////////


  public static final LevelSet m_LevelSet = new LevelSet();

  public RobotContainer() {
    configureBindings();
  }

  SwerveInputStream DriveVelAngular = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> Controle_0.getLeftY() * -1,
      () -> Controle_0.getLeftX() * -1)
      .withControllerRotationAxis(Controle_0::getRightX)
      .deadband(SwerveConstants.ZonaMorta)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = DriveVelAngular.copy().withControllerHeadingAxis(
      Controle_0::getRightX,
      Controle_0::getRightY)
      .headingWhile(true);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(DriveVelAngular);
  Command driveRobotOriented = drivebase.driveRobotOriented(DriveVelAngular);

  private void configureBindings() {
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    new Trigger(() -> Controle_0.getLeftTriggerAxis() > 0.25).onTrue(m_ball_intake);
    new Trigger(() -> Controle_0.getRightTriggerAxis() > 0.25).onTrue(m_ball_shooter);
  }
}