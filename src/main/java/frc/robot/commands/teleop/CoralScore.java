package frc.robot.commands.teleop;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class CoralScore extends Command {
  private final ArmSubsystem arm;
  private final ElevatorSubsystem elevator; // objeto do ShooterSubsystem
  private final XboxController Controle_0;
  // boolean finished = false;

  static ArrayList<Integer> parametros = new ArrayList<>();

  public CoralScore(ArmSubsystem arm, ElevatorSubsystem Elevator, XboxController Controle_0, int level) {

    this.elevator = Elevator;
    this.arm = arm;
    this.Controle_0 = Controle_0;
    addRequirements(elevator);
    
    switch (level) {
      case 1:
        parametros.add(1); // GRAUS DO ELEVADOR INTERNO// .get(0)
        parametros.add(2); // GRAUS DO ELEVADOR EXTERNO// .get(1)
        parametros.add(3); // ANGULO DA GARRA //.get(2)
        parametros.add(4); // VELOCIDADE DA GARRA //.get(3)
      case 2:
        parametros.add(1); // GRAUS DO ELEVADOR INTERNO// .get(0)
        parametros.add(2); // GRAUS DO ELEVADOR EXTERNO// .get(1)
        parametros.add(3); // ANGULO DA GARRA //.get(2)
        parametros.add(4); // VELOCIDADE DA GARRA //.get(3)
      case 3:
        parametros.add(1); // GRAUS DO ELEVADOR INTERNO// .get(0)
        parametros.add(2); // GRAUS DO ELEVADOR EXTERNO// .get(1)
        parametros.add(3); // ANGULO DA GARRA //.get(2)
        parametros.add(4); // VELOCIDADE DA GARRA //.get(3)
      case 4:
        parametros.add(1); // GRAUS DO ELEVADOR INTERNO// .get(0)
        parametros.add(2); // GRAUS DO ELEVADOR EXTERNO// .get(1)
        parametros.add(3); // ANGULO DA GARRA //.get(2)
        parametros.add(4); // VELOCIDADE DA GARRA //.get(3)
    }
  }
  // Called when the command is initially scheduled.

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ElevatorSubsystem.setpoint();
    arm.setpoint();
    ElevatorSubsystem.intakepeca();
    ElevatorSubsystem.ligarelevador(parametros.get(0), parametros.get(1)); // MUDAR ISSO PARA GRAUS
    arm.angleset(parametros.get(2), parametros.get(3));
  }

  public static boolean condicao_coral() {
    if (ElevatorSubsystem.angulo_elevador_interno() == parametros.get(0) &&
        ElevatorSubsystem.angulo_elevador_externo() == parametros.get(1)) {
      // finished = true;
      return true;
    } else {
      return false;
    }

  }

  @Override
  public void end(boolean interrupted) {
    ElevatorSubsystem.setpoint();
    arm.setpoint();
    // TALVEZ ANDAR PARA TRÁZ
  }

  @Override
  public boolean isFinished() {
    if (condicao_coral() == true &&
        AlinhamentoReef.condicao_reef() == true) { // A CONDIÇÃO PARA TERMINAR O COMANDO DEVE SER O TERMINO DA EXECUÇÃO
                                                   // DO EXECUTE
                                                   // E O ALINHAMENTO COMPLETO DO LADO ESCOLHIDO DO REEF
      return true; // Termina o comando
    }
    return false;
  }

}
