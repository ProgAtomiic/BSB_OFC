// package frc.robot.commands.teleop;

// import java.util.ArrayList;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;

// public class BallTakeOut extends Command {
//   private final ArmSubsystem arm;
//   private final ElevatorSubsystem elevator; // objeto do ShooterSubsystem
//   private final XboxController Controle_0;
//   // boolean finished = false;

//   static ArrayList<Integer> parametros = new ArrayList<>();

//   public BallTakeOut(ArmSubsystem arm, ElevatorSubsystem Elevator, XboxController Controle_0, int ball_level) {
//     this.elevator = Elevator;
//     this.arm = arm;
//     this.Controle_0 = Controle_0;
//     addRequirements(elevator);
//     switch (ball_level) {
//       case 1:
//         parametros.add(1); // GRAUS DO ELEVADOR EXTERNO// .get(0)
//         parametros.add(2); // GRAUS DO ELEVADOR EXTERNO// .get(1)

//       case 2:
//         parametros.add(1); // GRAUS DO ELEVADOR EXTERNO// .get(0)
//         parametros.add(2); // GRAUS DO ELEVADOR EXTERNO// .get(1)

//     }
//   }
//   // Called when the command is initially scheduled.

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     /// ANGULA GARRA NUMA POSIÇÃO MAIS ALTA EM RELÇÃO A BOLA
//     /// SOBE ELEVADOR INTERNO NO MAXIMO
//     /// SOBE ELEVADOR EXTERNO NO ANGULO PARÂMETROS
//     /// ANDA PARA FRENTE
//     /// DESCE O ANGULO DA GARRA
//     /// DESCE O ELEVADOR EXTERNO PARA PARAMETROS
//     /// ANDA PARA TRAZ

//     arm.angleset(360, 0.2);
//     ElevatorSubsystem.settop_interno();
//     ElevatorSubsystem.ligaelevador_externo(parametros.get(0)); // MUDAR ISSO PARA GRAUS
//     /// ANDA PARA FRENTE
//     arm.angleset(180, -0.2);
//     ElevatorSubsystem.ligaelevador_externo(parametros.get(1)); // MUDAR ISSO PARA GRAUS
//     /// ANDA PARA TRAZ
//   }

//   @Override
//   public void end(boolean interrupted) {
//     ElevatorSubsystem.setpoint();
//     arm.setpoint();
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }

// }
