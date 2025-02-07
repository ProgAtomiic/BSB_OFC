// package frc.robot.commands.teleop;

// import edu.wpi.first.wpilibj2.command.Command;

// public class AlinhamentoReef extends Command {
//     String lado = null;

//     public AlinhamentoReef(char lado) {

//         switch (lado) {
//             case 'D':
//                 lado = 'D';
//             case 'E':
//                 lado = 'E';
//         }
//     }

//     @Override
//     public void execute() {
//         // ALINHA CORRETAMENTE DE ACORDO COM A VARIAVEL LADO
//     }

//     public static boolean condicao_reef() {
//         if (true) { // A CONDIÇÃO DEVE SER SE O REEF FOI ALINHADO COMPLETAMENTE
//             return true;
//         } else {
//             return false;
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         // NADA
//     }

//     @Override
//     public boolean isFinished() {
//         if (CoralScore.condicao_coral() == true
//                 && condicao_reef() == true) { // A CONDIÇÃO PARA TERMINAR O COMANDO DEVE SER O TERMINO DA EXECUÇÃO DO
//                                               // EXECUTE
//                                               // E O ALINHAMENTO COMPLETO DO LADO ESCOLHIDO DO REEF
//             return true; // Termina o comando
//         }
//         return false;
//     }

// }
