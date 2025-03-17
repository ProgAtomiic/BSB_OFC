package frc.robot.commands.teleop.Alga;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaSubsystem;

public class BallIntake extends Command {
  double linhacomalga = -45;
  private final AlgaSubsystem intakealga;
  private final XboxController Controle_0;

  double TempoIntake;
  double TempoSeguranca;
  double TempoSegurancaIntake;
  boolean Terminado;
  boolean Interrompido;
  int Case;

  

  static ArrayList<Integer> parametros = new ArrayList<>();

  public BallIntake(AlgaSubsystem Intakealga, XboxController Controle_0) {
    this.intakealga = Intakealga;
    this.Controle_0 = Controle_0;
    addRequirements(intakealga);
  }

  @Override
  public void initialize() {
    TempoSeguranca = Timer.getFPGATimestamp();
    Terminado = false;
    Case = 0;
    linhacomalga = -45;
  }

  @Override
  public void execute() {
    System.out.println(AlgaSubsystem.EncoderLinha());

  
    // LIGA INTAKE
    if (AlgaSubsystem.EncoderLinha() < -60 && Case != 3) {
      AlgaSubsystem.ligaintake(-0.8);
    } else if (AlgaSubsystem.EncoderLinha() < -110 && Case == 3) {
      AlgaSubsystem.ligaintake(-0.5);
    } else {
      AlgaSubsystem.desligaintake();
    }
    // MOVE A LINHA
    if (AlgaSubsystem.TemAlga() == false) {
      if (Timer.getFPGATimestamp() - TempoSeguranca > 7) {
        Case = 1; // SOBE O INTAKE PARA A POSIÇÃO 0
      } else {
        Case = 2; // DESCE O INTAKE PARA A POSIÇÃO 120
        TempoSegurancaIntake = Timer.getFPGATimestamp();
      }
    } else if (AlgaSubsystem.TemAlga() == true) {
      Case = 3; // SOBE O INTAKE PARA A POSIÇÃO 0
    }
    switch (Case) {
      case 1:
        AlgaSubsystem.linha(0);
        if (AlgaSubsystem.EncoderLinha() < -5 && AlgaSubsystem.EncoderLinha() > 5) {
          Terminado = true;
        }
        break;
      case 2:
        AlgaSubsystem.linha(-110);
        break;
      case 3:
        if (Timer.getFPGATimestamp() - TempoSegurancaIntake > 0.5) {
          AlgaSubsystem.linha(linhacomalga);
        }

        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    AlgaSubsystem.linha(0);
    AlgaSubsystem.desligaintake();
    Case = 0;
    // Voltar para o angulo

  }

  @Override
  public boolean isFinished() {
    return Terminado;
  }
}
