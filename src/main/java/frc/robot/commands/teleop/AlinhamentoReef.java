package frc.robot.commands.teleop;

import java.util.ArrayList;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PIDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.AlinhamentoConstants;
public class AlinhamentoReef extends Command {
    private final ArmSubsystem arm;
    private final ElevatorSubsystem elevator; // objeto do ShooterSubsystem
    private final XboxController Controle_0;
    private final int Level;
    private final SwerveSubsystem SwerveSubsystem;
    private final PIDSubsystem PIDSubsystem;
    // boolean finished = false;
    public int Lado;
    public double Lado2;
    static ArrayList<Integer> Parametros = new ArrayList<>();

    public AlinhamentoReef(ArmSubsystem arm, ElevatorSubsystem Elevator, XboxController Controle_0, int level, SwerveSubsystem SwerveSubsystem, PIDSubsystem PIDSubsystem) {
        this.elevator = Elevator;
        this.arm = arm;
        this.Controle_0 = Controle_0;
        this.Level = level;
        this.SwerveSubsystem = SwerveSubsystem;
        this.PIDSubsystem = PIDSubsystem;
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        switch (Level) {
            case 0:
            //NADA
            break;
            case 1:
                Parametros.add(1); // GRAUS DO ELEVADOR INTERNO// .get(0)
                Parametros.add(2); // GRAUS DO ELEVADOR EXTERNO// .get(1)
                Parametros.add(3); // ANGULO DA GARRA //.get(2)
                Parametros.add(4); // VELOCIDADE DA GARRA //.get(3)
                Parametros.add(5); // DISTANCIA DE ALINHAR NO REEF //.get(4)
            break;
            case 2:
                Parametros.add(1); // GRAUS DO ELEVADOR INTERNO// .get(0)
                Parametros.add(2); // GRAUS DO ELEVADOR EXTERNO// .get(1)
                Parametros.add(3); // ANGULO DA GARRA //.get(2)
                Parametros.add(4); // VELOCIDADE DA GARRA //.get(3)
                Parametros.add(5); // DISTANCIA DE ALINHAR NO REEF //.get(4)
            break;
            case 3:
                Parametros.add(1); // GRAUS DO ELEVADOR INTERNO// .get(0)
                Parametros.add(2); // GRAUS DO ELEVADOR EXTERNO// .get(1)
                Parametros.add(3); // ANGULO DA GARRA //.get(2)
                Parametros.add(4); // VELOCIDADE DA GARRA //.get(3)
                Parametros.add(5); // DISTANCIA DE ALINHAR NO REEF //.get(4)
            break;
            case 4:
                Parametros.add(1); // GRAUS DO ELEVADOR INTERNO// .get(0)
                Parametros.add(2); // GRAUS DO ELEVADOR EXTERNO// .get(1)
                Parametros.add(3); // ANGULO DA GARRA //.get(2)
                Parametros.add(4); // VELOCIDADE DA GARRA //.get(3)
                Parametros.add(5); // DISTANCIA DE ALINHAR NO REEF //.get(4)
            break;
        }

        switch(Lado){
            case 0:
            //NADA 
            break;
            case 1:
                Lado2 = AlinhamentoConstants.Esquerda;
            break;

            case 2:
                Lado2 = AlinhamentoConstants.Direita;
            break;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //PID




        PIDSubsystem.PID(null, null, null, Lado2, null);
        






        //SwerveSubsystem.getSwerveDrive().driveFieldOriented(new ChassisSpeeds(2, 3, 3));
       
       
       
       
       
       
       
       
       
       
       
       
       
        ElevatorSubsystem.setpoint();
        arm.setpoint();
        ElevatorSubsystem.intakepeca();
        ElevatorSubsystem.ligarelevador(Parametros.get(0), Parametros.get(1)); // MUDAR ISSO PARA GRAUS
        arm.angleset(Parametros.get(2), Parametros.get(3));
    }

    public static boolean condicao_coral() {
        if (ElevatorSubsystem.angulo_elevador_interno() == Parametros.get(0) &&
                ElevatorSubsystem.angulo_elevador_externo() == Parametros.get(1)) {
            // finished = true;
            return true;
        } else {
            return false;
        }

    }

    public static boolean condicao_reef() {
        if (true) { // A CONDIÇÃO DEVE SER SE O REEF FOI ALINHADO COMPLETAMENTE
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // NADA
    }

    @Override
    public boolean isFinished() {
        if (CoralScore.condicao_coral() == true
                && condicao_reef() == true) { // A CONDIÇÃO PARA TERMINAR O COMANDO DEVE SER O TERMINO DA EXECUÇÃO DO
                                              // EXECUTE
                                              // E O ALINHAMENTO COMPLETO DO LADO ESCOLHIDO DO REEF
            return true; // Termina o comando
        }
        return false;
    }

}
