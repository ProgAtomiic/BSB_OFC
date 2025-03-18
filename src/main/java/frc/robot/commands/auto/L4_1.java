package frc.robot.commands.auto;

import java.util.ArrayList;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LevelSet;
import frc.robot.commands.teleop.Reef.ResetLevel;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class L4_1 extends Command {
  private final ElevatorSubsystem Elevator;
  private final ArmSubsystem armSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final LevelSet levelSet;
  private XboxController Controller;

  double alvo = 67.8;
  int Parte1;
  int Parte2;
  boolean terminado;
  boolean Validacao;
  int Case;
  double TempoAlinhamento;
  double TempoGarra;
  boolean CondicaoGarra;

  static ArrayList<Integer> parametros = new ArrayList<>();

  public L4_1(ElevatorSubsystem Elevator, ArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem, LevelSet levelSet, XboxController Controller) {
    this.Elevator = Elevator;
    this.armSubsystem = armSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.levelSet = levelSet;
    this.Controller = Controller;
    addRequirements(Elevator, armSubsystem);
  }

  @Override
  public void initialize() {
    TempoAlinhamento = 0;
    Parte1 = 1;
    Parte2 = 1;
    levelSet.setLevelTerminado(false);
    CondicaoGarra = false;

  }

  @Override
  public void execute() {
    

    if (levelSet.isAlinhamentoTerminado() == false) {
      switch (Parte1) {
        case 1:
          ElevatorSubsystem.PIDNoFFMaisFF(alvo);

          if (ElevatorSubsystem.GetPosicaoElevador() > 7) {
            ArmSubsystem.angleset(170);
            if (ArmSubsystem.angleget() > 165 && ArmSubsystem.angleget() < 175 && ElevatorSubsystem.GetPosicaoElevador() > 66 && ElevatorSubsystem.GetPosicaoElevador() < 68) {
              Parte1 = 0;
              levelSet.setLevelTerminado(true);
              
            }
          }
          break;
      }
    } 
  }

  @Override
  public void end(boolean interrupted) {
    Case = 0;
    levelSet.setLevel2(false);
    ArmSubsystem.angleset(90);
  }

  @Override
  public boolean isFinished() {
    return levelSet.isLevelTerminado();

  }

}
