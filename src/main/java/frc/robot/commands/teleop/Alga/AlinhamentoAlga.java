
package frc.robot.commands.teleop.Alga;

import java.util.ArrayList;

// import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class AlinhamentoAlga extends Command {
    
    // region VARIÁVEIS
    private final XboxController Controle_0;
    private final XboxController Controle_2;
    private final SwerveSubsystem SwerveSubsystem;
    private final AlgaSubsystem AlgaSubsystem;

    private VisionSubsystem Vision = new VisionSubsystem();
        int CaseAlinhamento = 1;
    
        public double AlvoX;
        public boolean Alinhado;
        public double DistanciaAlinhar;
        public static double AjusteLateral;
        public double AjusteDistancia;
        public double AjusteDistancia2;
        double RotacaoRobo;
        double AjusteRotacao;
        int tag;
    
        static ArrayList<Integer> Parametros = new ArrayList<>();
    
        PIDController PIDY = new PIDController(1.9, 0, 0);
        PIDController PIDRotacao = new PIDController(0.04, 0, 0.0);
    
    
        public AlinhamentoAlga(XboxController Controle_0, XboxController Controle_2, SwerveSubsystem SwerveSubsystem, AlgaSubsystem AlgaSubsystem) {
            this.Controle_0 = Controle_0;
            this.Controle_2 = Controle_2;
            this.SwerveSubsystem = SwerveSubsystem;
            this.AlgaSubsystem = AlgaSubsystem;

            if (DriverStation.getAlliance().isPresent()){
                if(DriverStation.getAlliance().get() == Alliance.Red){
                    tag = 3;
                }
                else if (DriverStation.getAlliance().get() == Alliance.Blue){
                    tag = 16;
                }
                }
        addRequirements(SwerveSubsystem);
    }

    @Override
    public void initialize() {
        Alinhado = false;
        
    }

    @Override
    public void execute() {
        System.out.print("rotacaorobo:");

        if (Vision.ArducamGetLatestResult().hasTargets()) {
            var bestTarget = Vision.ArducamGetLatestResult().getBestTarget();
            // System.out.print(Units.radiansToDegrees(bestTarget.bestCameraToTarget.getRotation().getZ()));
            SmartDashboard.putNumber("arducam", Units.radiansToDegrees(bestTarget.bestCameraToTarget.getRotation().getZ()));

            if (bestTarget != null && bestTarget.getFiducialId() == tag) { // Avoid NullPointerException
                
                double YAtual = bestTarget.getBestCameraToTarget().getY();

                if (Units.radiansToDegrees(bestTarget.bestCameraToTarget.getRotation().getZ()) < 0) {
                    RotacaoRobo = Units.radiansToDegrees(bestTarget.bestCameraToTarget.getRotation().getZ()) + 360;
                } else {
                    RotacaoRobo = Units.radiansToDegrees(bestTarget.bestCameraToTarget.getRotation().getZ());
                }
                if ((YAtual > -0.05 && YAtual < 0.05)   &&  (RotacaoRobo > 179.5 &&  RotacaoRobo < 180.5) ) { // Alignment within range
                        AjusteLateral = 0;
                        Alinhado = true;
                        } else { // Adjust if not aligned
                        AjusteLateral = PIDY.calculate(YAtual, 0); // TODO: set correct target
                        AjusteRotacao = PIDRotacao.calculate(RotacaoRobo, 180);
                        SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(0, -AjusteLateral, AjusteRotacao));
                        }

                        } 
                        else {
                                // Alinhado = true;
                    }
                
            }else {
                // Alinhado = true;
            }
        }
            
    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
        //no ultimo case colocar para descer o elavador para 0, e aqui desligar o elevador
    }

    @Override
    public boolean isFinished() {
        return Alinhado;

    }
}