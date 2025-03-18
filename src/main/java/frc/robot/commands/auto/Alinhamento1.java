
package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;

// import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlinhamentoConstants;
import frc.robot.LevelSet;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Alinhamento1 extends Command {

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

    PIDController PIDX = new PIDController(2.7, 0.0, 0);
    PIDController PIDY = new PIDController(3.9, 0.00, 0.0001);
    PIDController PIDRotacao = new PIDController(0.045, 0.0, 0.000005);

    public Alinhamento1(ArmSubsystem Arm, ElevatorSubsystem Elevator, XboxController Controle_0, XboxController Controle_2, SwerveSubsystem SwerveSubsystem, LevelSet LevelSet) {
        this.Elevator = Elevator;
        this.Arm = Arm;
        this.Controle_0 = Controle_0;
        this.Controle_2 = Controle_2;
        this.SwerveSubsystem = SwerveSubsystem;
        this.LevelSet = LevelSet;
        addRequirements(SwerveSubsystem);

    }

    @Override
    public void initialize() {
        CaseAlinhamento = 1;
        LevelSet.selectLevel(4);
        AlvoY = AlinhamentoConstants.Direita;
        startEncoder = Math.abs(SwerveSubsystem.DistanciaEncoder());

    }

    @Override
    public void execute() {

        switch (CaseAlinhamento) { // Bate no reef

            case 1: // Anda pra tras
                double DistanciaEncoder = Math.abs(SwerveSubsystem.DistanciaEncoder());
                if ((Math.abs(DistanciaEncoder - startEncoder) > 0.55)
                        && (Math.abs(DistanciaEncoder - startEncoder) < 0.65)) {
                    SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
                    DistanciaAlinhar = DistanciaEncoder - startEncoder;
                    CaseAlinhamento = 2;

                } else {
                    SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(
                        MathUtil.clamp(PIDX.calculate(Math.abs(DistanciaEncoder - startEncoder), 0.6), -0.8, 0.8),
                        0, 0));
                }
                break;

            case 2:// Anda para o lado
                if (Vision.LimeGetLatestResult().hasTargets()) {
                    var bestTarget = Vision.LimeGetLatestResult().getBestTarget();
                    if ((bestTarget != null)) {

                        YAtual = bestTarget.getBestCameraToTarget().getY();
                        if (Math.abs(YAtual) > 0.12 && Math.abs(YAtual) < 0.14) {
                        SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(0,0, 0));
                        Alinhado = true;
                        
                        } else {
                            AjusteLateral = PIDY.calculate(YAtual, AlvoY);
                            SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(0, MathUtil.clamp(AjusteLateral, -0.3, 0.3), 0));
                        }
                    }
                }
                break;

        }

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return Alinhado;

    }
}

