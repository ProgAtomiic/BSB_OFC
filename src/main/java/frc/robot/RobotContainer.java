// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
// import frc.robot.commands.teleop.AlinhamentoReef;
// import frc.robot.commands.teleop.BallIntake;
// import frc.robot.commands.teleop.BallShooter;
// import frc.robot.commands.teleop.BallTakeOut;
// import frc.robot.commands.teleop.CoralScore;
// import frc.robot.subsystems.AlgaSubsystem;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;



public class RobotContainer {
  
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private static final XboxController Controle_0 = new XboxController(OperatorConstants.kDriverControllerPort);

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // public static final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  // public static final ArmSubsystem m_amr = new ArmSubsystem();
  // public static final CoralScore m_levelsetL1 = new CoralScore(m_amr, m_elevator, Controle_0, 1);
  // public static final CoralScore m_levelsetL2 = new CoralScore(m_amr, m_elevator, Controle_0, 2);
  // public static final CoralScore m_levelsetL3 = new CoralScore(m_amr, m_elevator, Controle_0, 3);
  // public static final CoralScore m_levelsetL4 = new CoralScore(m_amr, m_elevator, Controle_0, 4);
  // ////////////////////////////////////////////////////////////////////////////////////////////////////
  // public static final AlinhamentoReef m_alinhamentoreefDIREITA = new AlinhamentoReef('D');
  // public static final AlinhamentoReef m_alinhamentoreefESQUERDA = new AlinhamentoReef('E');
  // ////////////////////////////////////////////////////////////////////////////////////////////////////
  // public static final AlgaSubsystem m_intake_alga = new AlgaSubsystem();
  // public static final BallIntake m_ball_intake = new BallIntake(m_intake_alga, Controle_0);
  // public static final BallShooter m_ball_shooter = new BallShooter(m_intake_alga, Controle_0);
  // ////////////////////////////////////////////////////////////////////////////////////////////////////
  // public static final BallTakeOut m_ball_take_out_baixo = new BallTakeOut(m_amr, m_elevator, Controle_0, 1);
  // public static final BallTakeOut m_ball_take_out_cima = new BallTakeOut(m_amr, m_elevator, Controle_0, 2);

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
      .allianceRelativeControl(false);

  SwerveInputStream driveDirectAngle = DriveVelAngular.copy().withControllerHeadingAxis(
      Controle_0::getRightX,
      Controle_0::getRightY)
      .headingWhile(true);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(DriveVelAngular);
  Command driveRobotOriented = drivebase.driveRobotOriented(DriveVelAngular);

  private void configureBindings() {
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    //Controle_0.getAButton().onTrue(drivebase.getPathCommand("Teste"));

    // new Trigger(Axis3::get).onTrue(m_ShootDefault);
    // new Trigger(Axis2::get).onTrue(m_IntakeDefault);
    // Botao5.whileTrue(m_AjusteAngulo_Distancia);

    /// TA RUIM, MUDAR.
    /// MELHOR COLOCAR ESSA CONDIÇÃO DE VER ALGUMA TAR DO REEF DENTRO DE UM CICLO
    /// QUE RODA IDEPENDENTE
    /// DOS BOTÕES PRA FAZER O CONTORLE VIBRAR.
    /// A SEGUINTE CONDIÇÃO DEVE SER FEITA DEITOR DO PRÓPRIO COMANDO, APENAS O
    /// VIBRAR DO CONTROLE DEVE
    /// SER FEITO FORA.
    /// TEM QUE ADICIONAR A FUNÇÃO DO DETECTOR DE PRESENÇA PRO CANO TAMBÉM.
    // if (true) { // SE O ROBÔ ESTIVER VENDO ALGUMA TAG DE REEF
    //   new Trigger(() -> Controle_0.getRawButton(0)).onTrue(m_levelsetL1);
    //   new Trigger(() -> Controle_0.getRawButton(1)).onTrue(m_levelsetL2);
    //   new Trigger(() -> Controle_0.getRawButton(2)).onTrue(m_levelsetL3);
    //   new Trigger(() -> Controle_0.getRawButton(3)).onTrue(m_levelsetL4);
    // }
    // ////////////////////////////////////////////////////////////////////////////////////////////////////
    // new Trigger(() -> Controle_0.getRawButton(4)).onTrue(m_alinhamentoreefESQUERDA);
    // new Trigger(() -> Controle_0.getRawButton(5)).onTrue(m_alinhamentoreefDIREITA);
    // ////////////////////////////////////////////////////////////////////////////////////////////////////
    // new Trigger(() -> Controle_0.getRawButton(6)).onTrue(m_ball_intake);
    // new Trigger(() -> Controle_0.getRawButton(7)).onTrue(m_ball_shooter);
    // ////////////////////////////////////////////////////////////////////////////////////////////////////
    // new Trigger(() -> Controle_0.getRawButton(12)).onTrue(m_ball_take_out_cima);
    // new Trigger(() -> Controle_0.getRawButton(13)).onTrue(m_ball_take_out_baixo);
   

  // public Command getAutonomousCommand() {
  //   return drivebase.getAutonomousCommand("Auto1");
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
}