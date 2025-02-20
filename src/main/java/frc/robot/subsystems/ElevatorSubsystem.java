package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Preferences;


public class ElevatorSubsystem extends SubsystemBase {
    // region VARIÁVEIS
    int meuInt = Preferences.getInt("minhaChave", 0);
    
    static DigitalInput FimdeCurso1 = new DigitalInput(0); // BAIXO INTERNO
    static DigitalInput FimdeCurso2 = new DigitalInput(0); // CIMA INTERNO

    static SparkMax ElevadorExtD = new SparkMax(15, MotorType.kBrushless);
    static RelativeEncoder EncoderExtD = ElevadorExtD.getEncoder();
    static DutyCycleEncoder EncoderAbsExt = new DutyCycleEncoder(0);



    static DigitalInput TemCoral = new DigitalInput(0);


    public static PIDController PID = new PIDController(0, 0, 0);

    private static final ElevatorFeedforward FeedforwardD = new ElevatorFeedforward(0.36, 0.11, 3.07, 0.02);

    public static double Ajuste;
    public static double AjusteFF;
    public static double AjustePID;

    private static double UltimaPosicao = 0;
    private static int ContadorRotacao = 0;


    // endregion

    public ElevatorSubsystem() {
    }


    public static void PIDNoFF(double Setpoint) {// TODO: NEGATIVAR O MOTOR(se precisar)
        Ajuste = FeedforwardD.calculate(PID.calculate(GetPosicaoElevador(), Setpoint));
        
        ElevadorExtD.setVoltage(MathUtil.clamp(Ajuste, -1, 1));

    }

    public static void PIDMaisFF(double Setpoint) {// TODO: NEGATIVAR O MOTOR(se precisar)
        Ajuste = (FeedforwardD.calculate(0.1) + PID.calculate(GetPosicaoElevador(), Setpoint));

        ElevadorExtD.setVoltage(MathUtil.clamp(Ajuste, -1, 1));
    }

    public static void PIDNoFFMaisFF(double Setpoint) {// TODO: NEGATIVAR O MOTOR(se precisar)

        AjustePID = PID.calculate(GetPosicaoElevador(), Setpoint);
        AjusteFF = FeedforwardD.calculate(AjustePID);

        ElevadorExtD.setVoltage(MathUtil.clamp((AjustePID + AjusteFF), -1, 1));
    }

    public static void DesligarElevador() { // TODO: NEGATIVAR O MOTOR(se precisar)
        ElevadorExtD.set(0);
        ElevadorExtD.set(0);
    }

    public static double GetAnguloEncoder() {
        return GetPosicaoElevador();

    }

    public static double GetPosicaoElevador() {
        double PosicaoAtual = EncoderAbsExt.get();// Retorna em rotações (0 a 1)

        // Detecta se houve uma rotação completa
        if (PosicaoAtual < 0.1 && UltimaPosicao > 0.9) { // Se a rotação atual for baixa e ultima foi quase 360, quer dizer que deu uma volta
            ContadorRotacao++; // Passou de 360° para 0°
        
        } else if (PosicaoAtual > 0.9 && UltimaPosicao < 0.1) { // Se a rotação atual for quase 360 e a ultima foi muito baixa
    
            ContadorRotacao--; // Passou de 0° para 360° no sentido inverso
        }

        UltimaPosicao = PosicaoAtual;

        // Calcula a posição total em rotações
        return ContadorRotacao + PosicaoAtual;
    }

    public static void DescerLimite() {// TODO: NEGATIVAR O MOTOR(se precisar)
        if (FimdeCurso1.get() == false) {
            ElevadorExtD.set(-0.2);
        }
    }

    public static void SubirLimite() {// TODO: NEGATIVAR O MOTOR(se precisar)
        if (FimdeCurso2.get() == false) {
            ElevadorExtD.set(0.2);
        }
    }

    public static boolean Descido() {
        return FimdeCurso1.get();
    }

    public static boolean TemCoral() {
        return TemCoral();
    }

}