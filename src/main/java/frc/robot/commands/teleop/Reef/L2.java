package frc.robot.commands.teleop.Reef;

import java.util.ArrayList;

import javax.naming.ldap.Control;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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


  double alvo = 30;
  double TempoAlinhamento;
  int Parte1;
  int Parte2;
  boolean Validacao;
  int Case;

  // boolean finished = false;

  public L2(ElevatorSubsystem Elevator, ArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem, LevelSet levelSet, XboxController Controller ) {
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
    Parte1 = 1;
    Parte2 = 1;
  }

  @Override
  public void execute() { // TODO: COMANDO NAO ESTA TERMINANDO


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
            if (ArmSubsystem.angleget() > 165 && ArmSubsystem.angleget() < 175 && ElevatorSubsystem.GetPosicaoElevador() > 27 && ElevatorSubsystem.GetPosicaoElevador() < 33) {
              Parte1 = 0;
            }
          }
          break;

      }
    } else if (levelSet.isAlinhamentoTerminado() == true) {

      
      switch (Parte2) {
        
        case 1:
          if (ArmSubsystem.angleget() > 157 && ArmSubsystem.angleget() < 170 ) {
            ArmSubsystem.angleset(170);
            Parte2 = 2;
          } else {
            ArmSubsystem.angleset(170);
          }
          break;

        case 2:
          if (ArmSubsystem.angleget() > 80 && ArmSubsystem.angleget() < 95) {
            TempoAlinhamento = Timer.getFPGATimestamp();
            Parte2 = 3;
          } else {
            ArmSubsystem.angleset(90);
          }
          break;

        case 3:
          ArmSubsystem.angleset(90);
          
          
          
          
          
        if (Timer.getFPGATimestamp() - TempoAlinhamento > 2 && Timer.getFPGATimestamp() - TempoAlinhamento < 2.1) {
            ElevatorSubsystem.LigarMotorArm(0.5);
            swerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(1, 0, 0));

          
  
          } else if (Timer.getFPGATimestamp() - TempoAlinhamento > 2.1) {

            swerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
            ElevatorSubsystem.LigarMotorArm(0);
            // levelSet.setLevelTerminado(true);

          }

      }

    }

    // if (levelSet.isAlinhamentoTerminado()) {
    // switch (CaseL2) {
    // case 0: // Move elevator up to at least 5 before starting arm movement
    // ElevatorSubsystem.PIDNoFFMaisFF(10);
    // if (ElevatorSubsystem.GetPosicaoElevador() > 5) {
    // CaseL2 = 1; // Once at safe height, move to next CaseL2
    // }
    // break;

    // case 1: // Start moving the arm while the Elevator finishes moving
    // ArmSubsystem.angleset(140); // Move arm up
    // if (Math.abs(ElevatorSubsystem.GetPosicaoElevador() - 10) < 0.05) {
    // CaseL2 = 2; // Move to next CaseL2 once Elevator reaches 10
    // }
    // break;

    // case 2: // Move Elevator down while the arm is still going up
    // ElevatorSubsystem.PIDNoFFMaisFF(0);
    // if (ArmSubsystem.angleget() > 100) { // Allow Elevator to go down before arm
    // reaches 140
    // CaseL2 = 3;
    // }
    // break;
    // }
    // } else {

    // switch (CaseL2) {
    // case 3: // Move arm down to 0 and start motor
    // ArmSubsystem.angleset(0);
    // // timerStart = Timer.getFPGATimestamp(); // Start a timer
    // CaseL2 = 4;
    // break;

    // case 4: // Run arm motor until arm reaches 0
    // ElevatorSubsystem.LigarMotorArm(0.5);
    // levelSet.setLevelTerminado(true);

    // break;

    // }
    // }

  }

  @Override
  public void end(boolean interrupted) {
    Case = 0;
    Validacao = false;
    levelSet.setLevel2(true);
    ArmSubsystem.angleset(90);
    new ResetLevel(Elevator, armSubsystem).schedule();

  }

  @Override
  public boolean isFinished() {
    return levelSet.isLevelTerminado();
  }

}
