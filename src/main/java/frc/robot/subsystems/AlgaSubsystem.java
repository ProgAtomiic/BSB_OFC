package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaSubsystem extends SubsystemBase {

    static SparkMax linha = new SparkMax(21, SparkMax.MotorType.kBrushless);
    static SparkMax intake = new SparkMax(22, SparkMax.MotorType.kBrushless);

    static RelativeEncoder linhaEncoder = linha.getEncoder();

    static DigitalInput TemAlga = new DigitalInput(1);
    static PIDController PID = new PIDController(0.05, 0, 0);

    public static void reset_motor() {
        linhaEncoder.setPosition(0);
    }

    public static void linha(double rotacao) {
        double velocidade = PID.calculate(linhaEncoder.getPosition(), rotacao);
        double vel_max = 0.8;
        linha.set(MathUtil.clamp(velocidade, -0.8, 0.8));
    }

    public static double EncoderLinha() {
        return linha.getEncoder().getPosition();
    }

    public static void linhasobe(double velocidade) {
        linha.set(velocidade);
    }

    public static void ligaintake(double velocidade) {
        intake.set(velocidade);
    }

    public static void desligaintake() {
        intake.set(0);
    }

    public static void desligalinha() {
        linha.set(0);
    }

    public static boolean TemAlga() {
        return TemAlga.get();
    }
}
