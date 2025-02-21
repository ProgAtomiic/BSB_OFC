
// package frc.robot.commands.teleop;

// import java.util.ArrayList;

// // import java.util.ArrayList;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.AlinhamentoConstants;
// import frc.robot.LevelSet;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.PIDSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;

// public class AlinhamentoReef extends Command {
//     // region VARIÁVEIS
//     private final ArmSubsystem Arm;
//     private final ElevatorSubsystem Elevator;
//     private final XboxController Controle_0;
//     private final XboxController Controle_2;
//     private final int Level;
//     private final SwerveSubsystem SwerveSubsystem;
//     private final String Lado;
//     private final frc.robot.LevelSet LevelSet;
//     private final VisionSubsystem Vision = new VisionSubsystem();
//     int Teste;

//     public double AlvoX;
//     public double Servo;
//     public boolean Alinhado;
//     private final Timer Timer = new Timer();
//     Timer TempoServo = new Timer();
//     double DistanciaAlinhar;
//     double AjusteRotacao;
//     double AjusteDistancia;
//     double AjusteDistancia2;

//     static ArrayList<Integer> Parametros = new ArrayList<>();

//     PIDController PIDX = new PIDController(0, 0, 0);
//     PIDController PIDY = new PIDController(0, 0, 0);

//     // endregion

//     public AlinhamentoReef(ArmSubsystem Arm, ElevatorSubsystem Elevator, XboxController Controle_0,
//             XboxController Controle_2,
//             SwerveSubsystem SwerveSubsystem, String Lado, int Level, LevelSet LevelSet) {
//         this.Elevator = Elevator;
//         this.Arm = Arm;
//         this.Controle_0 = Controle_0;
//         this.Controle_2 = Controle_2;
//         this.Level = Level;
//         this.SwerveSubsystem = SwerveSubsystem;
//         this.Lado = Lado;
//         this.LevelSet = LevelSet;
//         addRequirements(Elevator, Arm, SwerveSubsystem);
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//         Timer.start();
//         switch (Level) {
//             case 1:
//                 Parametros.add(2); // GRAUS DO ELEVADOR EXTERNO// .get(0)

//                 Parametros.add(3); // ANGULO INICIAL DA GARRA (para subir o elevador) //.get(1)
//                 Parametros.add(3); // ANGULO FINAL DA GARRA (para pontuar)//.get(2)

//                 Parametros.add(4); // VELOCIDADE DA GARRA//.get(3)

//                 Parametros.add(4); // VELOCIDADE DA GARRA (SERVO) //.get(4)

//                 break;
//             case 2:
//                 Parametros.add(2); // GRAUS DO ELEVADOR EXTERNO// .get(0)

//                 Parametros.add(3); // ANGULO INICIAL DA GARRA (para subir o elevador) //.get(1)
//                 Parametros.add(3); // ANGULO FINAL DA GARRA (para pontuar)//.get(2)

//                 Parametros.add(4); // VELOCIDADE DA GARRA//.get(3)

//                 break;
//             case 3:
//                 Parametros.add(2); // GRAUS DO ELEVADOR EXTERNO// .get(0)

//                 Parametros.add(3); // ANGULO INICIAL DA GARRA (para subir o elevador) //.get(1)
//                 Parametros.add(3); // ANGULO FINAL DA GARRA (para pontuar)//.get(2)

//                 Parametros.add(4); // VELOCIDADE DA GARRA//.get(3)

//                 break;
//             case 4:
//                 Parametros.add(2); // GRAUS DO ELEVADOR EXTERNO// .get(0)

//                 Parametros.add(3); // ANGULO INICIAL DA GARRA (para subir o elevador) //.get(1)
//                 Parametros.add(3); // ANGULO FINAL DA GARRA (para pontuar)//.get(2)

//                 Parametros.add(4); // VELOCIDADE DA GARRA//.get(3)

//                 break;
//         }
//         if (Lado == "Esquerda") {
//             AlvoX = AlinhamentoConstants.Esquerda;
//             LevelSet.RumbleControle(Controle_2, RumbleType.kLeftRumble, Timer.get());
//         } else if (Lado == "Direita") {
//             AlvoX = AlinhamentoConstants.Direita;
//             Controle_2.setRumble(RumbleType.kRightRumble, 1);
//         }
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {

//         double DistanciaAtualLime = Vision.DistanciaDaTagLime();
//         double pitchAtual = VisionSubsystem.Limelight.getLatestResult().getBestTarget().pitch;

//         boolean AlinhadoDistancia = (SwerveSubsystem.DistanciaEncoder() == DistanciaAlinhar);
//         boolean alinhadoRotacao = (pitchAtual == AlvoX);
//         // ElevatorSubsystem.PIDMaisFF(Parametros.get(0)); //Talvez tenha que fazer um
//         // if

//         // region ALINHAMENTO
//         if (Level > 1) {
//             switch (Teste) {
//                 case 1: // Alinha o chassi e levanta a garra para cima
//                     if (alinhadoRotacao) { // Coloca o alvo do alinhamento de distancia, zera o ajuste do eixo X
//                         DistanciaAlinhar = (9 - DistanciaAtualLime) + SwerveSubsystem.DistanciaEncoder();
//                         AjusteRotacao = 0;
//                         Teste = 2;

//                     } else { // Se não estiver alinhado na rotação, ajusta a e levanta a garra

//                         AjusteRotacao = PIDX.calculate(pitchAtual, AlvoX);

//                         // Levanta a garra para seu angulo inicial
//                         // Arm.angleset(Parametros.get(1)); // Para cima

//                         // Faz o ajuste do robô
//                         SwerveSubsystem.getSwerveDrive().driveFieldOriented( // TODO: pode ser que tenha que ser robot
//                                 new ChassisSpeeds(AjusteRotacao, 0, 0));

//                     }
//                     break;
//                 case 2: // Anha o chassi a distancia, mantendo o comando do braço
//                     if (AlinhadoDistancia) {// Zera o ajuste de distancia
//                         AjusteDistancia = 0; // TODO: talvez precise colocar o method de rodar swerve
//                         Teste = 3;
//                     } else {// Se não estiver alinhado na distância, ajusta frente/trás, mantém o comando do
//                             // braço

//                         AjusteDistancia = PIDY.calculate(SwerveSubsystem.DistanciaEncoder(), DistanciaAlinhar);

//                         SwerveSubsystem.getSwerveDrive().driveFieldOriented(new ChassisSpeeds(0, AjusteDistancia, 0));
//                     }

//                     break;

//                 case 3: // Coloca a garra na posicao de pontuar
//                     Teste = 4;
//                     // if (Arm.angleget() == Parametros.get(2)) { //Levantou o braço
//                     // Teste = 4;
//                     // } else { //Levanta o braço
//                     // Arm.angleset(Parametros.get(2));
//                     // }
//                     break;

//                 case 4:
//                     if (Level == 4) {
//                         if (SwerveSubsystem.DistanciaEncoder() == 9) {
//                             AjusteDistancia2 = 0;
//                             Alinhado = true;

//                         } else { // Vai andar para trás ligando o servo motor
//                             // Liga o servo motor
//                             // Arm.LigarServo(0.1);
//                             AjusteDistancia2 = AjusteDistancia = PIDY.calculate(SwerveSubsystem.DistanciaEncoder(), (SwerveSubsystem.DistanciaEncoder()-9));

//                             SwerveSubsystem.getSwerveDrive().driveFieldOriented( // TODO: pode ser que tenha que ser

//                                     new ChassisSpeeds(0, AjusteDistancia2, 0)); // por uma certa distancia?
//                         }
//                     } else {

//                         if (Arm.angleget() == 0) { // Descer a garra
//                             Alinhado = true;

//                         } else { // Vai descer a garra ligando o servo motor
//                             Arm.angleset(0);
//                             Arm.LigarServo(0.1);
//                         }
//                     }
//                     break;
//                 default:
//                     break;
//             }

//         } else if (Level == 1) {
//             if (Arm.angleget() == Parametros.get(0) && ElevatorSubsystem.GetPosicaoElevador() == Parametros.get(0)) {
//                 TempoServo.start();
//                 if (TempoServo.get() < 5) {
//                     Arm.LigarServo(0.1);

//                 } else {
//                     Arm.DesligarServo();
//                     Alinhado = true;

//                 }
//             } else {
//                 Arm.angleset(Parametros.get(0));
//                 ElevatorSubsystem.PIDMaisFF(Parametros.get(0));
//             }
//         }
//         // endregion

//         // Cancela o comando
//         if ((Controle_0.getRawButton(5)) && LevelSet.Contador() >= 1) {
//             Alinhado = true;
//         }

//     }

//     @Override
//     public void end(boolean interrupted) {

//         Timer.stop();
//         LevelSet.ResetContador();
//         ElevatorSubsystem.DescerLimite();
//         Arm.angleset(0);
//         SwerveSubsystem.getSwerveDrive().driveFieldOriented(new ChassisSpeeds(0, 0, 0));
//         LevelSet.LadoEscolhido = "";
//         LevelSet.SetLevel(0);
//     }

//     @Override
//     public boolean isFinished() {
//         return Alinhado;

//     }
// }