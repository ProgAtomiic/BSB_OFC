package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    public static DigitalInput FimdeCursoBaixo = new DigitalInput(7); 
    public static DigitalInput FimdeCursoCima = new DigitalInput(8); 
    public DigitalInput TemCoral = new DigitalInput(2);

    static SparkMax Elevador = new SparkMax(12, MotorType.kBrushless);
    public static RelativeEncoder EncoderElevador = Elevador.getEncoder();
    static SparkMax ArmMotor = new SparkMax(15, SparkMax.MotorType.kBrushless);

    static PIDController PID = new PIDController(0.3,0.0008, 0.01);

    private final static ElevatorFeedforward FeedforwardD = new ElevatorFeedforward(0.36, 0.13, 3.07, 0.01);

    public ElevatorSubsystem() {
    }

    public static void set(double Vel) {
        Elevador.set(Vel);
    }

    public static void PIDNoFFMaisFF(double Setpoint) {

        double AjustePID = PID.calculate(GetPosicaoElevador(), Setpoint);
        double AjusteFF = FeedforwardD.calculate(AjustePID);
        double soma_ajuste = AjusteFF + AjustePID;
        
        Elevador.setVoltage(MathUtil.clamp(soma_ajuste, -3, 7));

    }

    public static void DesligarElevador() { 
        Elevador.set(0);
    }

    public static void ResetAnguloEncoder() {
        EncoderElevador.setPosition(0);

    }

    public static double GetPosicaoElevador() {

        return EncoderElevador.getPosition();
    }

    public static void LigarMotorArm(double Velocidade) {
        ArmMotor.set(Velocidade);
    }

    public static boolean FimdeCursoBaixo() {
        return FimdeCursoBaixo.get();
    }

    public static boolean FimdeCursoCima() {
        return FimdeCursoCima.get();
    }

    public boolean TemCoral() {
        return TemCoral.get();
    }

    @Override
public void periodic() {
    // var currentCommand = getCurrentCommand();
    // if (currentCommand != null) {
    //     System.out.println("[ElevatorSubsystem] Running Command: " + currentCommand.getName());
    // }

}

}