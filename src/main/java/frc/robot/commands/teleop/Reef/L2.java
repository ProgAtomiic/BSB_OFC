package frc.robot.commands.teleop.Reef;

import java.util.ArrayList;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.LevelSet;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class L2 extends Command {
  private final ElevatorSubsystem Elevator;
  private final ArmSubsystem armSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final LevelSet levelSet;
  private XboxController Controller;


  double alvo = 15
  ;
  boolean Terminado;
  double TempoAlinhamento;

  int Parte1;
  int Parte2;
  boolean Validacao;
  int Case;

  boolean CondicaoGarra;
  private static final XboxController Controle_0 = new XboxController(OperatorConstants.ControlePrincipal);

  

  // boolean finished = false;

  public L2(ElevatorSubsystem Elevator, ArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem, LevelSet levelSet, XboxController Controller) {
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
    Parte2 = 1;
    
    CondicaoGarra = false;
  }

  @Override
  public void execute() {

    System.out.println();
    
    // if (Controller.getLeftStickButton()) {
    //   Case = 1;
    // }

    // switch (Case) {
    //   case 1:
    //     Validacao = true;
    //     break;

    // }
    
    if (levelSet.isAlinhamentoTerminado() == false) {
      switch (Parte1) {
        case 1:
          
          ElevatorSubsystem.PIDNoFFMaisFF(alvo);

          if (ElevatorSubsystem.GetPosicaoElevador() > 7) {
            ArmSubsystem.angleset(170);
            if (ArmSubsystem.angleget() > 160 && ArmSubsystem.angleget() < 177
                && ElevatorSubsystem.GetPosicaoElevador() > 13 && ElevatorSubsystem.GetPosicaoElevador() < 16) {
              Parte1 = 0;
              
            }
          }
          break;

      }
    } else if (levelSet.isAlinhamentoTerminado() == true) {

      switch (Parte2) {

        case 1:
          if (ArmSubsystem.angleget() > 164 && ArmSubsystem.angleget() < 176) {
            ArmSubsystem.angleset(170);
            levelSet.setParte1Terminado(true);

            Parte2 = 2;
          } else {
            ArmSubsystem.angleset(170);
          }
          break;

        case 2:
           if (Controle_0.getLeftStickButton() == true){
            CondicaoGarra = true;
          }

          //ALTERAACAO 5
          if (ArmSubsystem.angleget() > 80 && ArmSubsystem.angleget() < 100) {
            TempoAlinhamento = Timer.getFPGATimestamp();
            Parte2 = 3;
          } else if (CondicaoGarra == true){
            ArmSubsystem.angleset(90);
          }
          else {
          ArmSubsystem.angleset(170);
        }


          break;
        // ALTERACAO 3
        
        case 3:
          ArmSubsystem.angleset(90);
          if (ArmSubsystem.angleget() > 80 && ArmSubsystem.angleget() < 100) {
            levelSet.setLevelTerminado(true);
          }
          // if (Timer.getFPGATimestamp() - TempoAlinhamento > 1 && Timer.getFPGATimestamp() - TempoAlinhamento < 1.3) {
          //   ElevatorSubsystem.LigarMotorArm(0.5);

          // } else if (Timer.getFPGATimestamp() - TempoAlinhamento > 1.3) {

          //   ElevatorSubsystem.LigarMotorArm(0);
          //   levelSet.setLevelTerminado(true);


          // }

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
