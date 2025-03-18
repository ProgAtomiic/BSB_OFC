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

public class L4_2 extends Command {
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

  public L4_2(ElevatorSubsystem Elevator, ArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem, LevelSet levelSet,
      XboxController Controller) {
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

   
      switch (Parte2) {

        case 1:
          if (ArmSubsystem.angleget() > 160 && ArmSubsystem.angleget() < 175) {
            ArmSubsystem.angleset(170);
            levelSet.setParte1Terminado(true);
            Parte2 = 3;

          } else {
            ArmSubsystem.angleset(170);
          }

          break;

        case 3:
          if (ArmSubsystem.angleget() > 80 && ArmSubsystem.angleget() < 95) {
            TempoAlinhamento = Timer.getFPGATimestamp();
            Parte2 = 4;
          } else if (Controller.getLeftStickButton() == true) {
            ArmSubsystem.angleset(90);
          }


          break;
        case 4:
          ArmSubsystem.angleset(90);

          if (Timer.getFPGATimestamp() - TempoAlinhamento > 0.1 && Timer.getFPGATimestamp() - TempoAlinhamento < 0.5) {
            ElevatorSubsystem.LigarMotorArm(0.5);
            swerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(0.3, 0, 0));


          } else if (Timer.getFPGATimestamp() - TempoAlinhamento > 1) {
            ElevatorSubsystem.LigarMotorArm(0);
            swerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
            levelSet.setLevelTerminado(true);

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
    ArmSubsystem.angleset(90);
    // new ResetLevel(Elevator, armSubsystem).schedule();
  }

  @Override
  public boolean isFinished() {
    return levelSet.isLevelTerminado();

  }

}
