// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.teleop.AlinhamentoReef;
import frc.robot.commands.teleop.IntakeCoral;
import frc.robot.commands.teleop.arm_command;
//import frc.robot.commands.teleop.AlinhamentoReef;
//import frc.robot.commands.teleop.BallIntake;
//import frc.robot.commands.teleop.BallShooter;
//import frc.robot.commands.teleop.BallTakeOut;
//import frc.robot.commands.teleop.CoralScore;
//import frc.robot.subsystems.AlgaSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private static final XboxController Controle_0 = new XboxController(OperatorConstants.ControlePrincipal);
  private static final XboxController Controle_2 = new XboxController(OperatorConstants.ControleSecundario);



  /*
   * /////////////////////////////////////////////////////////////////////////////
   * ///////////////////////
   * public static final BallTakeOut m_ball_take_out_baixo = new
   * BallTakeOut(m_amr, m_elevator, Controle_0, 1);
   * public static final BallTakeOut m_ball_take_out_cima = new BallTakeOut(m_amr,
   * m_elevator, Controle_0, 2);
   */
  // public static final AlgaSubsystem m_intake_alga = new AlgaSubsystem();
  // public static final BallIntake m_ball_intake = new BallIntake(m_intake_alga,
  // Controle_0);
  // public static final BallShooter m_ball_shooter = new
  // BallShooter(m_intake_alga, Controle_0);
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  public static final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();

  public static final arm_command m_arm_0 = new arm_command(m_ArmSubsystem, Controle_0, 0);
  public static final arm_command m_arm_45 = new arm_command(m_ArmSubsystem, Controle_0, 45);
  public static final arm_command m_arm_90 = new arm_command(m_ArmSubsystem, Controle_0, 90);
  public static final arm_command m_arm_135 = new arm_command(m_ArmSubsystem, Controle_0, 135);
  public static final arm_command m_arm_set = new arm_command(m_ArmSubsystem, Controle_0, -1);

  public static final LevelSet m_LevelSet = new LevelSet();
  public static final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();

  public RobotContainer() {
    NamedCommands.registerCommand("IntakeCoral", new IntakeCoral(m_ElevatorSubsystem));
    NamedCommands.registerCommand("AlinhamentoDireita", new AlinhamentoReef(m_ArmSubsystem, m_ElevatorSubsystem, Controle_0, Controle_2, drivebase, "Direita", 0, m_LevelSet)); //ESCOLHER O LADO
    NamedCommands.registerCommand("AlinhamentoEsquerda", new AlinhamentoReef(m_ArmSubsystem, m_ElevatorSubsystem, Controle_0, Controle_2, drivebase, "Esquerda", 0, m_LevelSet)); //ESCOLHER O LADO


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
    // Controle_0.getAButton().whenTrue(drivebase.getPathCommand("Teste")); ARRUMAR

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
    /*
     * if (true) { // SE O ROBÔ ESTIVER VENDO ALGUMA TAG DE REEF
     * new Trigger(() -> Controle_0.getRawButton(0)).onTrue(m_levelsetL1);
     * new Trigger(() -> Controle_0.getRawButton(1)).onTrue(m_levelsetL2);
     * new Trigger(() -> Controle_0.getRawButton(2)).onTrue(m_levelsetL3);
     * new Trigger(() -> Controle_0.getRawButton(3)).onTrue(m_levelsetL4);
     * }
     * /////////////////////////////////////////////////////////////////////////////
     * ///////////////////////
     * // new Trigger(() ->
     * Controle_0.getRawButton(4)).onTrue(m_alinhamentoreefESQUERDA);
     * // new Trigger(() ->
     * Controle_0.getRawButton(5)).onTrue(m_alinhamentoreefDIREITA);
     * /////////////////////////////////////////////////////////////////////////////
     * ///////////////////////
     * new Trigger(() -> Controle_0.getRawButton(6)).onTrue(m_ball_intake);
     * new Trigger(() -> Controle_0.getRawButton(7)).onTrue(m_ball_shooter);
     * /////////////////////////////////////////////////////////////////////////////
     * ///////////////////////
     * new Trigger(() -> Controle_0.getRawButton(12)).onTrue(m_ball_take_out_cima);
     * new Trigger(() -> Controle_0.getRawButton(13)).onTrue(m_ball_take_out_baixo);
     * /////////////////////////////////////////////////////////////////////////////
     * ///////////////////////
     * 
     */
    /*
     * new Trigger(() -> Controle_0.getRawButton(5)).whileTrue(m_ball_intake);
     * new Trigger(() -> Controle_0.getRawButton(6)).whileTrue(m_ball_shooter);
     * new Trigger(() -> Controle_0.getRawButton(1)).whileTrue(m_ball_intake);
     * new Trigger(() -> Controle_0.getRawButton(2)).whileTrue(m_ball_shooter);
     */

    // TODO: DAQUI PRA BAIXO TEM CHANCE DE FUNCIONAR:
    // new Trigger(() -> Controle_0.getRawButton(1)).whileTrue(m_arm_0);
    // new Trigger(() -> Controle_0.getRawButton(2)).whileTrue(m_arm_45);
    // new Trigger(() -> Controle_0.getRawButton(3)).whileTrue(m_arm_90);
    // new Trigger(() -> Controle_0.getRawButton(4)).whileTrue(m_arm_135);
    // new Trigger(() -> Controle_0.getRawButton(5)).whileTrue(m_arm_set);

    // new JoystickButton(Controle_0, 5).onTrue(new InstantCommand(() ->
    // m_LevelSet.SetLado("Esquerda")));
    // new JoystickButton(Controle_0, 6).onTrue(new InstantCommand(() ->
    // m_LevelSet.SetLado("Direita")));
    // // Botões para escolher o nível (só funciona se um lado foi escolhido)
    // new JoystickButton(Controle_2, 1).onTrue(new InstantCommand(() ->
    // m_LevelSet.SetLevel(1)));
    // new JoystickButton(Controle_2, 2).onTrue(new InstantCommand(() ->
    // m_LevelSet.SetLevel(2)));
    // new JoystickButton(Controle_2, 3).onTrue(new InstantCommand(() ->
    // m_LevelSet.SetLevel(3)));
    // new JoystickButton(Controle_2, 4).onTrue(new InstantCommand(() ->
    // m_LevelSet.SetLevel(4)));

    if (m_LevelSet.TemLevel() == true) {
      new AlinhamentoReef(m_ArmSubsystem, m_ElevatorSubsystem, Controle_0, Controle_2, drivebase,
        m_LevelSet.GetLado(), m_LevelSet.GetLevel(), m_LevelSet).schedule();

    } 



    
    // public Command getAutonomousCommand() {
    // return drivebase.getAutonomousCommand("Auto1");
    // }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

  }
}