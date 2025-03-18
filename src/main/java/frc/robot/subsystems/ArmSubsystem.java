package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    public int angulo_alvo = 0;

    static SparkMax arm = new SparkMax(11, SparkMax.MotorType.kBrushless);
    static RelativeEncoder encoder = arm.getEncoder();

    static PIDController PID = new PIDController(0.06, 0.00, 0.00005);
    private final static ArmFeedforward feedforward = new ArmFeedforward(0.5, 0.7, 0.45);
    
    public ArmSubsystem() {}

    public static void reset_motor() {
        encoder.setPosition(0);
    }

    public static void angleset(double angulo) {
        double vel_max = 1.5;
        double velocidade = PID.calculate(angleget(), angulo);
        double velocidade1 = feedforward.calculate(Math.toRadians(angleget()), velocidade);
        double soma_velocidade = velocidade + velocidade1;

        arm.setVoltage(MathUtil.clamp(soma_velocidade, -1.5, 1.5));
    }

    public static void set(double velocidade) {
        arm.set(velocidade);
    }

    public static double angleget() {
        return (encoder.getPosition() * 14.4);
    }

    public void stoparm() {
        arm.set(0);
    }

    @Override
    public void periodic() {
        System.out.println("angulo garra"+angleget()); 
    }
}
