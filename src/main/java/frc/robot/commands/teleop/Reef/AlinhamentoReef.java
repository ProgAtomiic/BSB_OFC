
package frc.robot.commands.teleop.Reef;

import edu.wpi.first.math.MathUtil;

// import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlinhamentoConstants;
import frc.robot.Constants;
import frc.robot.LevelSet;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlinhamentoReef extends Command {

    // region VARIÁVEIS
    private final ArmSubsystem Arm;
    private final ElevatorSubsystem Elevator;
    private final XboxController Controle_0;
    private final XboxController Controle_2;
    double starttime;

    // private final int Level;
    private final SwerveSubsystem SwerveSubsystem;
    private Rotation2d targetRotation;

    // private final String Lado;
    private final LevelSet LevelSet;
    private final VisionSubsystem Vision = new VisionSubsystem();
    int CaseAlinhamento;

    public double AlvoY;
    public double Servo;
    public boolean Alinhado;
    double DistanciaAlinhar;

    public double AjusteRotacao;
    double AjusteLateral;
    double AjusteDistancia;
    boolean TemDistanciaAlinhar;

    double RotacaoAtual;

    public double RotacaoRobo;
    static double yaw;
    public static double pitch;

    double Encoder;

    double YAtual;

    double TesteX;

    double startTime;

    double startEncoder;

    double Tolerancia;

    double TempoSeguranca;
    



    PIDController PIDX = new PIDController(2.7, 0.0, 0);
    PIDController PIDY = new PIDController(3.9, 0.00, 0.0001);
    // PIDController PIDY = new PIDController(4.3, 0.01, 0.);
    PIDController PIDRotacao = new PIDController(0.045, 0.0, 0.000005);


    public AlinhamentoReef(ArmSubsystem Arm, ElevatorSubsystem Elevator, XboxController Controle_0,
            XboxController Controle_2, SwerveSubsystem SwerveSubsystem, LevelSet LevelSet) {
        this.Elevator = Elevator;
        this.Arm = Arm;
        this.Controle_0 = Controle_0;
        this.Controle_2 = Controle_2;
        // this.Level = Level;
        this.SwerveSubsystem = SwerveSubsystem;
        // this.Lado = Lado;
        this.LevelSet = LevelSet;
        // addRequirements(SwerveSubsystem);

    }

    @Override
    public void initialize() {
        TempoSeguranca = 0;
        frc.robot.subsystems.SwerveSubsystem.VelocidadeSwerve = 1; //TODO: TIREI O 0.3 PARA TESTES DA PRIMEIRA PARTIDA
        CaseAlinhamento = 1;
        Tolerancia = 0.01;

        startEncoder = Math.abs(SwerveSubsystem.DistanciaEncoder());
        if (LevelSet.getLevelEscolhido() == 1){
            AlvoY = 0;
            Tolerancia = 0.0115;


        }
       else if (LevelSet.getLadoEscolhido() == "Esquerda") {
            AlvoY = AlinhamentoConstants.Esquerda;
            Tolerancia = 0.0115;

        } else if (LevelSet.getLadoEscolhido() == "Direita") {
            AlvoY = AlinhamentoConstants.Direita;
        }
       
        }

    

    @Override
    public void execute() {
        System.out.println("Casealinhamento" + CaseAlinhamento);


        // SmartDashboard.putNumber("ROTACAO ERRO: ", RotacaoRobo - 180);
        // SmartDashboard.putNumber("AJUSTE ROTACAO: ", AjusteRotacao);
        // SmartDashboard.putNumber("ROTACAO ATUAL", RotacaoRobo);


        // SmartDashboard.putNumber("X ERRO : ",Math.abs((Math.abs((SwerveSubsystem.DistanciaEncoder() - Encoder))- Math.abs(DistanciaAlinhar))));
        // SmartDashboard.putNumber("AJUSTE X : ", AjusteDistancia);
        SmartDashboard.putNumber("X ATUAL : ", SwerveSubsystem.DistanciaEncoder() - startEncoder);


        // SmartDashboard.putNumber("Y ERRO: ", YAtual - AlvoY);
        // SmartDashboard.putNumber("AJUSTE Y: ", AjusteLateral);
        // SmartDashboard.putNumber("Y ATUAL: ", YAtual);



        // System.out.println("caselinhamento" + CaseAlinhamento);
        SmartDashboard.putNumber("Y ATUAL: ", YAtual);



        switch (CaseAlinhamento) {
            
            case 1:
                double DistanciaEncoder =  Math.abs(SwerveSubsystem.DistanciaEncoder());
                if ((Math.abs(DistanciaEncoder - startEncoder) > 0.55) && (Math.abs(DistanciaEncoder - startEncoder) < 0.65 )){
                    SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
                    // System.out.println("pronto");
                    DistanciaAlinhar = DistanciaEncoder - startEncoder;
                    CaseAlinhamento  = 2;
                } else{
                    SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(MathUtil.clamp(PIDX.calculate(Math.abs(DistanciaEncoder - startEncoder), 0.6),-0.8, 0.8), 0, 0));
                }
                break;
            
                
            case 2:
                if (Vision.LimeGetLatestResult().hasTargets()) {
                     var bestTarget = Vision.LimeGetLatestResult().getBestTarget();
                     if ((bestTarget != null) ) { // Avoid NullPointerException


                         YAtual = bestTarget.getBestCameraToTarget().getY();
                         if (Math.abs(YAtual) > Math.abs(AlvoY) - Tolerancia && Math.abs(YAtual) < Math.abs(AlvoY) + Tolerancia) {
                            // SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
                            
                            Encoder = SwerveSubsystem.DistanciaEncoder();
                            CaseAlinhamento = 3;
                            TempoSeguranca = Timer.getFPGATimestamp();
                            DistanciaAlinhar = (bestTarget.getBestCameraToTarget().getX() - Math.copySign(0.02, bestTarget.getBestCameraToTarget().getX()));
 
                         } else {
                             AjusteLateral = PIDY.calculate(YAtual, AlvoY);
                             SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(0, MathUtil.clamp(AjusteLateral, -0.3, 0.3), 0));
                         }
                    } else {
                       Alinhado = true;
               }
                 } else {
                     Alinhado = true;
                 }
                 break;

            case 3:
    
                if (LevelSet.getLevelEscolhido() != 0 && !LevelSet.isComandoLevelIniciou()) {
                    LevelSet.startLevelCommand();
                    LevelSet.setComandoLevelIniciou(true);
                }

                if ((Math.abs((Math.abs((SwerveSubsystem.DistanciaEncoder() - Encoder)) - Math.abs(DistanciaAlinhar))) < 0.03) || (Timer.getFPGATimestamp() - TempoSeguranca  > 2)) {
                    AjusteDistancia = 0;
                    SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
                    CaseAlinhamento = 4; 

                }  
                
                else {
                    AjusteDistancia = PIDX.calculate(SwerveSubsystem.DistanciaEncoder() - Encoder, -DistanciaAlinhar);
                    SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(MathUtil.clamp(AjusteDistancia, -0.4, 0.4), 0, 0));
                }
                break;



            case 4:
                LevelSet.setAlinhamentoTerminado(true);
                Alinhado = LevelSet.isLevelTerminado();
                break;

            default:
                break;
        }




            // if ((Math.abs(YAtual) > 0.10 && Math.abs(YAtual) < 0.12)) {

            //     System.out.println("ALINHADO");
                
            //     AjusteLateral = 0;
            //     Encoder = SwerveSubsystem.DistanciaEncoder();
            //     CaseAlinhamento = 2;
            //     double starttime;

            // } else {
            //     startTime = Timer.getFPGATimestamp();
            //     AjusteLateral = PIDY.calculate(YAtual, AlvoY);
            //     SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(0, MathUtil.clamp(AjusteLateral, -2, 2), MathUtil.clamp(AjusteRotacao, -0.4, 0.4)));

            // }
    //     } else {
    //         Alinhado = true;
    // }
    // } else {
    //     Alinhado = true;
    // }
    // break;
    //     }
        }
    

    @Override
    public void end(boolean interrupted) {
        frc.robot.subsystems.SwerveSubsystem.VelocidadeSwerve = 1;
        LevelSet.resetSelection();
    }

    @Override
    public boolean isFinished() {
        return Alinhado;

    }
}
























// package frc.robot.commands.teleop.Reef;

// import edu.wpi.first.math.MathUtil;

// // import java.util.ArrayList;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.AlinhamentoConstants;
// import frc.robot.Constants;
// import frc.robot.LevelSet;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;

// public class AlinhamentoReef extends Command {

//     // region VARIÁVEIS
//     private final ArmSubsystem Arm;
//     private final ElevatorSubsystem Elevator;
//     private final XboxController Controle_0;
//     private final XboxController Controle_2;
//     double starttime;

//     // private final int Level;
//     private final SwerveSubsystem SwerveSubsystem;
//     private Rotation2d targetRotation;

//     // private final String Lado;
//     private final LevelSet LevelSet;
//     private final VisionSubsystem Vision = new VisionSubsystem();
//     int CaseAlinhamento;

//     public double AlvoY;
//     public double Servo;
//     public boolean Alinhado;
//     double DistanciaAlinhar;

//     public double AjusteRotacao;
//     double AjusteLateral;
//     double AjusteDistancia;
//     boolean TemDistanciaAlinhar;

//     double RotacaoAtual;

//     public double RotacaoRobo;
//     static double yaw;
//     public static double pitch;

//     double Encoder;

//     double YAtual;

//     double TesteX;

//     double startTime;

//     PIDController PIDX = new PIDController(1.5, 0, 0);
//     PIDController PIDY = new PIDController(3.7, 0.00, 0.0001);
//     // PIDController PIDY = new PIDController(4.3, 0.01, 0.);
//     PIDController PIDRotacao = new PIDController(0.045, 0.0, 0.000005);


//     public AlinhamentoReef(ArmSubsystem Arm, ElevatorSubsystem Elevator, XboxController Controle_0,
//             XboxController Controle_2, SwerveSubsystem SwerveSubsystem, LevelSet LevelSet) {
//         this.Elevator = Elevator;
//         this.Arm = Arm;
//         this.Controle_0 = Controle_0;
//         this.Controle_2 = Controle_2;
//         // this.Level = Level;
//         this.SwerveSubsystem = SwerveSubsystem;
//         // this.Lado = Lado;
//         this.LevelSet = LevelSet;
//         addRequirements(SwerveSubsystem);

//     }

//     @Override
//     public void initialize() {
//         CaseAlinhamento = 1;

//         if (LevelSet.getLadoEscolhido() == "Esquerda") {
//             AlvoY = AlinhamentoConstants.Esquerda;
//         } else if (LevelSet.getLadoEscolhido() == "Direita") {
//             AlvoY = AlinhamentoConstants.Direita;
//         }
//         if (LevelSet.getLevelEscolhido() == 1) {
//             AlvoY = 0;
//         }

//     }

//     @Override
//     public void execute() {


//         SmartDashboard.putNumber("ROTACAO ERRO: ", RotacaoRobo - 180);
//         SmartDashboard.putNumber("AJUSTE ROTACAO: ", AjusteRotacao);
//         SmartDashboard.putNumber("ROTACAO ATUAL", RotacaoRobo);


//         SmartDashboard.putNumber("X ERRO : ",Math.abs((Math.abs((SwerveSubsystem.DistanciaEncoder() - Encoder))- Math.abs(DistanciaAlinhar))));
//         SmartDashboard.putNumber("AJUSTE X : ", AjusteDistancia);
//         SmartDashboard.putNumber("X ATUAL : ", SwerveSubsystem.DistanciaEncoder() - Encoder);


//         SmartDashboard.putNumber("Y ERRO: ", YAtual - AlvoY);
//         SmartDashboard.putNumber("AJUSTE Y: ", AjusteLateral);
//         SmartDashboard.putNumber("Y ATUAL: ", YAtual);



//         System.out.println("caselinhamento" + CaseAlinhamento);

//         switch (CaseAlinhamento) {
            
//             case 1:
//                 if (Vision.LimeGetLatestResult().hasTargets()) {
//                     var bestTarget = Vision.LimeGetLatestResult().getBestTarget();
//                     if ((bestTarget != null) && (bestTarget.getBestCameraToTarget().getX() < 1.5 && bestTarget.getBestCameraToTarget().getX() > 0.6)) { // Avoid NullPointerException


//                         YAtual = bestTarget.getBestCameraToTarget().getY();

//                         if (Units.radiansToDegrees(
//                                 Vision.getTagRotation(Vision.LimeGetLatestResult().getBestTarget()).getZ()) < 0) {
//                             RotacaoRobo = Units.radiansToDegrees(
//                                     Vision.getTagRotation(Vision.LimeGetLatestResult().getBestTarget()).getZ()) + 360;
//                         } else {
//                             RotacaoRobo = Units.radiansToDegrees(
//                                     Vision.getTagRotation(Vision.LimeGetLatestResult().getBestTarget()).getZ());
//                         }
                        
//                         if ((RotacaoRobo > 179.4 && RotacaoRobo < 180.6)
//                                 && (Math.abs(YAtual) > 0.10 && Math.abs(YAtual) < 0.12)) {

                            
//                             System.out.println("ALINHADO");
                            
//                             AjusteLateral = 0;
//                             Encoder = SwerveSubsystem.DistanciaEncoder();
//                             DistanciaAlinhar = (bestTarget.getBestCameraToTarget().getX() - Math.copySign(0.07, bestTarget.getBestCameraToTarget().getX()));
//                             CaseAlinhamento = 2;
//                             double starttime;

//                         } else {
//                             startTime = Timer.getFPGATimestamp();
//                             AjusteLateral = PIDY.calculate(YAtual, AlvoY);
//                             if (YAtual < 1 && YAtual > -1){
//                                 AjusteRotacao = PIDRotacao.calculate(RotacaoRobo, 180);
//                             }
//                             SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(0, MathUtil.clamp(AjusteLateral, -2, 2), MathUtil.clamp(AjusteRotacao, -0.4, 0.4)));

//                         }
//                     } else {
//                         Alinhado = true;
//                 }
//                 } else {
//                     Alinhado = true;
//                 }
//                 break;
//             case 2:
    
//                 if (LevelSet.getLevelEscolhido() != 0 && !LevelSet.isComandoLevelIniciou()) {
//                     LevelSet.startLevelCommand();
//                     LevelSet.setComandoLevelIniciou(true);
//                 }

//                 if (Math.abs((Math.abs((SwerveSubsystem.DistanciaEncoder() - Encoder))- Math.abs(DistanciaAlinhar))) < 0.08) {
//                     AjusteDistancia = 0;
//                     SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
//                     CaseAlinhamento = 3; -+

//                 } else {
//                     AjusteDistancia = PIDX.calculate(SwerveSubsystem.DistanciaEncoder() - Encoder, -DistanciaAlinhar);
//                     SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(MathUtil.clamp(AjusteDistancia, -0.6, 0.6), 0, 0));
//                 }
//                 break;

//             case 3:
//                 LevelSet.setAlinhamentoTerminado(true);
//                 Alinhado = LevelSet.isLevelTerminado();
//                 break;

//             default:
//                 break;
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         LevelSet.resetSelection();
//     }

//     @Override
//     public boolean isFinished() {
//         return Alinhado;

//     }
// }