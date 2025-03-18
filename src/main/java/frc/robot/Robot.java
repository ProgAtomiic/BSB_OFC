// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.prefs.Preferences;

import org.photonvision.PhotonUtils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.Alinhamento2;
import frc.robot.commands.teleop.Reef.AlinhamentoReef;
import frc.robot.subsystems.AlgaSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Robot extends TimedRobot {
  Preferences prefs;

  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  

  private final XboxController Controle = new XboxController(0);
  private final VisionSubsystem Vision = new VisionSubsystem();
  // private final SwerveSubsystem Swerve = new SwerveSubsystem();

  public Robot() {
    m_robotContainer = new RobotContainer();
    
    ArmSubsystem.reset_motor();
    AlgaSubsystem.reset_motor();
    CameraServer.startAutomaticCapture();


  }

  @Override
  public void robotPeriodic() {


    if (ElevatorSubsystem.FimdeCursoBaixo() == false) {
      ElevatorSubsystem.EncoderElevador.setPosition(0);
    }
    if (ElevatorSubsystem.FimdeCursoCima() == false){
      ElevatorSubsystem.EncoderElevador.setPosition(66.859);
    }
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {


    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  //   if ((Controle).getStartButton() == true){
  //   AlgaSubsystem.linhasobe(0.2);
  // }
  //   else{
  //     AlgaSubsystem.linhasobe(0);
  //   }
  }
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
