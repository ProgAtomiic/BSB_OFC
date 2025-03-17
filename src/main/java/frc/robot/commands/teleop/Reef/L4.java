package frc.robot.commands.teleop.Reef;

import java.util.ArrayList;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LevelSet;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class L4 extends Command {
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

  static ArrayList<Integer> parametros = new ArrayList<>();

  public L4(ElevatorSubsystem Elevator, ArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem, LevelSet levelSet, XboxController Controller) {
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

  }

  @Override
  public void execute() {
    System.out.println(Timer.getFPGATimestamp() - TempoGarra);

    if (levelSet.isAlinhamentoTerminado() == false) {
      switch (Parte1) {
        case 1:
          ElevatorSubsystem.PIDNoFFMaisFF(alvo);

          if (ElevatorSubsystem.GetPosicaoElevador() > 7) {
            ArmSubsystem.angleset(170);
            if (ArmSubsystem.angleget() > 165 && ArmSubsystem.angleget() < 175
                && ElevatorSubsystem.GetPosicaoElevador() > 66 && ElevatorSubsystem.GetPosicaoElevador() < 68) {
              Parte1 = 0;
            }
          }
          break;

      }
    } else if (levelSet.isAlinhamentoTerminado() == true) {
      


      switch (Parte2) {

        case 1:
        if (ArmSubsystem.angleget() > 165 && ArmSubsystem.angleget() < 175) {
          ArmSubsystem.angleset(170);
          levelSet.setParte1Terminado(true);
          TempoGarra = Timer.getFPGATimestamp();
          Parte2 = 3;
  
        } else {
          ArmSubsystem.angleset(170);
        }

        break;
        case 2:
        // if(Timer.getFPGATimestamp() - TempoGarra > 3){ //TODO: TESTAR NO PIT PFVVVVV
        //   // ArmSubsystem.angleset(90);
        //   Parte2 = 3;
        // } 

        break;

        case 3:
          if (ArmSubsystem.angleget() > 80 && ArmSubsystem.angleget() < 95) {
            TempoAlinhamento = Timer.getFPGATimestamp();
            Parte2 = 4;
          } else if (Timer.getFPGATimestamp() - TempoGarra > 1){
            ArmSubsystem.angleset(90);
          }
          break;

        case 4:
          ArmSubsystem.angleset(90);
          if (Timer.getFPGATimestamp() - TempoAlinhamento > 1 && Timer.getFPGATimestamp() - TempoAlinhamento < 1.3) {
            ElevatorSubsystem.LigarMotorArm(0.5);

          } else if (Timer.getFPGATimestamp() - TempoAlinhamento > 1.3) {

            ElevatorSubsystem.LigarMotorArm(0);
            levelSet.setLevelTerminado(true);
            
          }

        default:
          break;
      }

    }
  }

  @Override
  public void end(boolean interrupted) {
    Case = 0;
    Validacao = false;
    levelSet.setLevel2(false);
    ArmSubsystem.angleset(90);
    new ResetLevel(Elevator, armSubsystem).schedule();
  }

  @Override
  public boolean isFinished() {
    return levelSet.isLevelTerminado();

  }

}
