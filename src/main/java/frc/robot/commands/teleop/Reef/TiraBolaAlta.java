package frc.robot.commands.teleop.Reef;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class TiraBolaAlta extends Command {
  private final AlgaSubsystem intakealga;
  private final ElevatorSubsystem Elevator;
  private final ArmSubsystem armSubsystem;


  double TempoAtual = Timer.getFPGATimestamp();
  double TempoIntake;
  double TempoSeguranca;
  double alvo = 40;
  int Case = 0;
  boolean Terminado;


  static ArrayList<Integer> parametros = new ArrayList<>();

  public TiraBolaAlta(AlgaSubsystem Intakealga, ElevatorSubsystem Elevator,ArmSubsystem armSubsystem) {
    this.Elevator = Elevator;
    this.intakealga = Intakealga;
    this.armSubsystem = armSubsystem;
    addRequirements(Elevator, armSubsystem);
  }

  @Override
  public void initialize() {

    Terminado = false;
  }

  //TODO: Caso continue dando muito conflito, para resolver deve finalizar (colocar um terminado = true) nos comandos --> ambos tira bola e intake de coral

  @Override
  public void execute() {   
    ElevatorSubsystem.PIDNoFFMaisFF(alvo);
    if (ElevatorSubsystem.GetPosicaoElevador() > 5){ 
      ArmSubsystem.angleset(95);
    }
  }
 
  @Override
  public void end(boolean interrupted) {
    ArmSubsystem.angleset(95);
    new ResetLevel(Elevator, armSubsystem).schedule();

  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
