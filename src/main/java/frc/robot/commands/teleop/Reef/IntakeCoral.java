// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.Reef;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {

  ElevatorSubsystem elevatorSubsystem;
  ArmSubsystem armSubsystem;
  VisionSubsystem Vision = new VisionSubsystem();
  int CaseCoral;
  boolean Validacao;
  double TempoIntake;
  boolean Terminado;
  double SetpointIntakeCoral = 24;
  int tag1;
  int tag2;

  public IntakeCoral(ElevatorSubsystem ElevatorSubsystem, ArmSubsystem armSubsystem) {
    this.elevatorSubsystem = ElevatorSubsystem;
    this.armSubsystem = armSubsystem;

    addRequirements(ElevatorSubsystem);
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        tag1 = 1;
        tag2 = 2;
      } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
        tag1 = 12;
        tag2 = 13;
      }
    }

  }

  @Override
  public void initialize() {
    Validacao = false;
    CaseCoral = 0;
    Terminado = false;
  }

  //TODO: Caso continue dando muito conflito, para resolver deve finalizar (colocar um terminado = true) nos comandos --> ambos tira bola e intake de coral
  @Override
  public void execute() {
    ArmSubsystem.angleset(0);

    // System.out.println(tag1);

    if (elevatorSubsystem.TemCoral() && CaseCoral == 0) {
      CaseCoral = 1;
    }

    switch (CaseCoral) {
      case 1:
        TempoIntake = Timer.getFPGATimestamp();
        CaseCoral = 2;
        break;

      case 2:
        Validacao = true;
        break;

    }

    if (Validacao) {

      if ((Timer.getFPGATimestamp() - TempoIntake) > 0.75 && (Timer.getFPGATimestamp() - TempoIntake) < 1.8) {
        ElevatorSubsystem.LigarMotorArm(-0.3);
      } else {
        ElevatorSubsystem.LigarMotorArm(0);
      }
      double tempoAtual = Timer.getFPGATimestamp();

      if ((tempoAtual - TempoIntake) > 0.5) {
        double posicaoAtual = ElevatorSubsystem.GetPosicaoElevador();

        if (Math.abs(posicaoAtual) < 0.08) {
           ElevatorSubsystem.DesligarElevador();
          //  ALTERACAO 5
           Terminado = true;

        } else {
          ElevatorSubsystem.PIDNoFFMaisFF(0);
        }
      }

    } else {
      if (!elevatorSubsystem.TemCoral()) {
        ElevatorSubsystem.PIDNoFFMaisFF(SetpointIntakeCoral);
      }
    }

  }

  @Override
  public void end(boolean interrupted) {
    ElevatorSubsystem.LigarMotorArm(0);
    new ResetLevel(elevatorSubsystem, armSubsystem).schedule();
    ArmSubsystem.reset_motor();

    Validacao = false;
    CaseCoral = 0;
    Terminado = false;

  }

  @Override
  public boolean isFinished() {
    return Terminado;
  }

}