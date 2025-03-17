// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.prefs.Preferences;

import org.photonvision.PhotonUtils;

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
import frc.robot.commands.teleop.Reef.AlinhamentoReef;
import frc.robot.commands.teleop.Reef.AutoDireita;
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

  }

  @Override
  public void robotPeriodic() {

    // System.out.println(Constants.Tag1());
    // System.out.println(Constants.Tag2());





    SmartDashboard.putNumber("Elevador", ElevatorSubsystem.GetPosicaoElevador());

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
    // int AUTO;

    // switch (AUTO) {
    //   PIDController PID = new PIDController(2.7, 0.0, 0);

    //   case 1:
    //   PIDController PID = new PIDController(2.7, 0.0, 0);

    //     if(SwerveSubsystem.DistanciaEncoder < 1){
    //       SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(new ChassisSpeeds(PID.calculate(SwerveSubsystem.DistanciaEncoder, 1), 0, 0)));

    //     }else{
    //       AUTO = 2;
    //     }
    //     break;

    //   case 2: 
    //   PIDController PID = new PIDController(2.7, 0.0, 0);

    //     if(SwerveSubsystem.DistanciaEncoder < 2){
    //       SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(new ChassisSpeeds(0, PID.calculate(SwerveSubsystem.DistanciaEncoder, 2), 0)));

    //     }else{
    //       double TempoAuto = Timer.getFPGATimestamp();
    //       AUTO = 3;
    //     }
    //     break;

    //   case 3: 
    //   PIDController PID = new PIDController(2.7, 0.0, 0);

    //     if(Timer.getFPGATimestamp() - TempoAuto < 1){
    //       SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(new ChassisSpeeds(1, 0, 0)));

    //     }else{
    //       AUTO = 4;
    //     }
    //     break;

    //   case 4:
      
    //     AutoDireita();
    //     AUTO = 5;
    //     break;

    //   case 5:
    //     PIDController PID = new PIDController(2.7, 0.0, 0);

    //     if(SwerveSubsystem.DistanciaEncoder < 2){
    //       SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(new ChassisSpeeds(0, PID.calculate(SwerveSubsystem.DistanciaEncoder, 2), 0)));
    //   }
      

          
    //   default:
    //     break;
    // }

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
