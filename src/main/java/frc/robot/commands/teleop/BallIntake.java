package frc.robot.commands.teleop;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaSubsystem;

public class BallIntake extends Command {
  private final AlgaSubsystem intakealga;
  private final XboxController Controle_0;
  double TempoAtual = Timer.getFPGATimestamp();
  double TempoIntake;
  double TempoSeguranca;
  int Case = 0;
  boolean Terminado;
  // boolean finished = false;

  static ArrayList<Integer> parametros = new ArrayList<>();

  
  public BallIntake(AlgaSubsystem Intakealga, XboxController Controle_0) {
    this.intakealga = Intakealga;
    this.Controle_0 = Controle_0;
    addRequirements(intakealga);
  }

  // Called when the command is initially scheduled.
  // Called every time the scheduler runs while the command is scheduled.




  @Override
  public void initialize() {
    TempoSeguranca = TempoAtual;

  }

  @Override
  public void execute() {
    AlgaSubsystem.linhaset(0.4);
    AlgaSubsystem.ligaintake(-0.7);
    if (AlgaSubsystem.TemAlga() == true) {
      Case = 1;
    } else if (TempoAtual - TempoSeguranca > 1){
      //voltar para o angulo
    }

    switch (Case) {
      case 1:
        TempoIntake = TempoAtual;
        Case = 2;
        break;
      case 2:
        if(TempoAtual - TempoIntake > 0.3){
          Terminado = true;
        } 
      break;
      default:
        break;
    }

  }

  @Override
  public void end(boolean interrupted) {
    AlgaSubsystem.desligaintake();
    AlgaSubsystem.desligalinha();
    //Voltar para o angulo
  
  }

  @Override
  public boolean isFinished() {
    return Terminado == true || AlgaSubsystem.TemAlga() == true;
  }

 }
