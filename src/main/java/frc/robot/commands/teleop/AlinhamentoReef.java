// package frc.robot.commands.teleop;

// import java.util.ArrayList;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.PIDSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
// import frc.robot.Constants.AlinhamentoConstants;

// public class AlinhamentoReef extends Command {
//     private final ArmSubsystem arm;
//     private final ElevatorSubsystem elevator; // objeto do ShooterSubsystem
//     private final XboxController Controle_0;
//     private int Level;
//     private final SwerveSubsystem SwerveSubsystem;
//     private final PIDSubsystem PIDSubsystem;
//     private final VisionSubsystem VisionSubsystem;
//     // boolean finished = false;
//     public int Lado;
//     public double Lado2;
//     public boolean Alinhado;
//     static ArrayList<Integer> Parametros = new ArrayList<>();

//     public AlinhamentoReef(ArmSubsystem arm, ElevatorSubsystem Elevator, XboxController Controle_0, int level, SwerveSubsystem SwerveSubsystem, PIDSubsystem PIDSubsystem, VisionSubsystem VisionSubsystem) {
//         this.elevator = Elevator;
//         this.arm = arm;
//         this.Controle_0 = Controle_0;
//         this.Level = level;
//         this.SwerveSubsystem = SwerveSubsystem;
//         this.PIDSubsystem = PIDSubsystem;
//         this.VisionSubsystem = VisionSubsystem;
//         addRequirements(elevator);
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//         if(Controle_0.getRawButton(1)){
//             Lado2 = AlinhamentoConstants.Esquerda;
//             Lado = 1;
//         } else if(Controle_0.getRawButton(2)){
//             Lado2 = AlinhamentoConstants.Direita;
//             Lado = 2;
//         } 

//         switch (Level) {
//             case 0:
//                 // NADA
//                 break;
//             case 1:
//                 Parametros.add(1); // GRAUS DO ELEVADOR INTERNO// .get(0)
//                 Parametros.add(2); // GRAUS DO ELEVADOR EXTERNO// .get(1)
//                 Parametros.add(3); // ANGULO DA GARRA //.get(2)
//                 Parametros.add(4); // VELOCIDADE DA GARRA //.get(3)
//                 Parametros.add(5); // DISTANCIA DE ALINHAR NO REEF //.get(4)
//                 break;
//             case 2:
//                 Parametros.add(1); // GRAUS DO ELEVADOR INTERNO// .get(0)
//                 Parametros.add(2); // GRAUS DO ELEVADOR EXTERNO// .get(1)
//                 Parametros.add(3); // ANGULO DA GARRA //.get(2)
//                 Parametros.add(4); // VELOCIDADE DA GARRA //.get(3)
//                 Parametros.add(5); // DISTANCIA DE ALINHAR NO REEF //.get(4)
//                 break;
//             case 3:
//                 Parametros.add(1); // GRAUS DO ELEVADOR INTERNO// .get(0)
//                 Parametros.add(2); // GRAUS DO ELEVADOR EXTERNO// .get(1)
//                 Parametros.add(3); // ANGULO DA GARRA //.get(2)
//                 Parametros.add(4); // VELOCIDADE DA GARRA //.get(3)
//                 Parametros.add(5); // DISTANCIA DE ALINHAR NO REEF //.get(4)
//                 break;
//             case 4:
//                 Parametros.add(1); // GRAUS DO ELEVADOR INTERNO// .get(0)
//                 Parametros.add(2); // GRAUS DO ELEVADOR EXTERNO// .get(1)
//                 Parametros.add(3); // ANGULO DA GARRA //.get(2)
//                 Parametros.add(4); // VELOCIDADE DA GARRA //.get(3)
//                 Parametros.add(5); // DISTANCIA DE ALINHAR NO REEF //.get(4)
//                 break;
//         }


//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//         // PID

//         if (Lado != 0 && Level != 0){
//         if (VisionSubsystem.DistanciaDaTag(frc.robot.subsystems.VisionSubsystem.Limelight) == Parametros.get(5)){ //Se tiver alinhado (frente/tras)
            
//             if (frc.robot.subsystems.VisionSubsystem.Limelight.getLatestResult().getBestTarget().pitch == Lado2){ //Se tiver alinhado (rotacional)
//                 System.out.println("Alinhado");
//                 Alinhado = true;
//                 // ElevatorSubsystem.setpoint();
//                 // arm.setpoint();
//                 // ElevatorSubsystem.intakepeca();
//                 // ElevatorSubsystem.ligarelevador(Parametros.get(0), Parametros.get(1)); // MUDAR ISSO PARA GRAUS
//                 // arm.angleset(Parametros.get(2), Parametros.get(3));

//             }else{
//                 SwerveSubsystem.getSwerveDrive().driveFieldOriented(new ChassisSpeeds(
//                     PIDSubsystem.PID(0, 0, 0, 
//                     Lado2, 
//                     frc.robot.subsystems.VisionSubsystem.Limelight.getLatestResult().getBestTarget().pitch), 
//                     0, 
//                     0));
//                     Alinhado = false;
//             }
//         }else{ //PID para ajustar frente/tras
//             SwerveSubsystem.getSwerveDrive().driveFieldOriented(new ChassisSpeeds(
//                 0, 
//                 PIDSubsystem.PID(0, 0, 0, 
//                 Parametros.get(5),
//                 VisionSubsystem.DistanciaDaTag(frc.robot.subsystems.VisionSubsystem.Limelight)), 
//                 0));
//                 Alinhado = false;
//         }
    
//     }
//     }
//     @Override
//     public void end(boolean interrupted) {
//         Level = 0;
//         Lado = 0;
//     }

//     @Override
//     public boolean isFinished() {
//         if (Alinhado == true){
//             return true; // Termina o comando
//         }
//         return false;
//     }

// }
