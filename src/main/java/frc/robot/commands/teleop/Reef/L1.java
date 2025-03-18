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

public class L1 extends Command {
  private final ElevatorSubsystem Elevator;
  private final ArmSubsystem armSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final LevelSet levelSet;
  private XboxController Controller;


  double alvo = 18;
  boolean Terminado;
  double TempoAlinhamento;

  int Parte1;
  int Parte2;
  boolean Validacao;
  int Case;

  // boolean finished = false;

  public L1(ElevatorSubsystem Elevator, ArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem, LevelSet levelSet, XboxController Controller) {
    this.Elevator = Elevator;
    this.armSubsystem = armSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.levelSet = levelSet;
    this.Controller = Controller;
    addRequirements(Elevator, armSubsystem);
  }

  @Override
  public void initialize() {
    levelSet.setLevelTerminado(false);
    TempoAlinhamento = 0;
    Parte1 = 1;
    Parte2 = 0;
  }

  @Override
  public void execute() {



    
    // if (Controller.getLeftStickButton()) {
    //   Case = 1;
    // }

    // switch (Case) {
    //   case 1:
    //     Validacao = true;
    //     break;

    // }

   
      switch (Parte1) {
        case 1:
          ElevatorSubsystem.PIDNoFFMaisFF(alvo);

          if (ElevatorSubsystem.GetPosicaoElevador() > 7) {
            ArmSubsystem.angleset(70);
            if (ArmSubsystem.angleget() > 65 && ArmSubsystem.angleget() < 75
                && ElevatorSubsystem.GetPosicaoElevador() > 17 && ElevatorSubsystem.GetPosicaoElevador() < 19) {
              Parte1 = 0;
              Parte2 = 1;

            }
          }
          break;

      }
    

      switch (Parte2) {

        case 1:
          if (ArmSubsystem.angleget() > 65
          
           && ArmSubsystem.angleget() < 75) {
            TempoAlinhamento = Timer.getFPGATimestamp();
            Parte2 = 2;
          } else {
            ArmSubsystem.angleset(70);
          }

          break;

        case 2:
          ArmSubsystem.angleset(70);
          if (ArmSubsystem.angleget() > 65 && ArmSubsystem.angleget() < 75) {

          {if (Timer.getFPGATimestamp() - TempoAlinhamento > 0.5 && Timer.getFPGATimestamp() - TempoAlinhamento < 0.8) {
            ElevatorSubsystem.LigarMotorArm(0.15);

          } else if (Timer.getFPGATimestamp() - TempoAlinhamento > 0.8) {

            ElevatorSubsystem.LigarMotorArm(0);
            levelSet.setLevelTerminado(true);

          }
          }
        }

        default:
          break;
      }

    
  }

  @Override
  public void end(boolean interrupted) {
    Case = 0;
    Validacao = false;
    levelSet.setLevel2(false);
    ArmSubsystem.angleset(70);
    new ResetLevel(Elevator, armSubsystem).schedule();

  }

  @Override
  public boolean isFinished() {
    return levelSet.isLevelTerminado();
  }

}
