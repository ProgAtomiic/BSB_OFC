package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaSubsystem extends SubsystemBase {
    static SparkMax linha = new SparkMax(7, SparkMax.MotorType.kBrushless);
    static SparkMax intake = new SparkMax(0, SparkMax.MotorType.kBrushless);

    RelativeEncoder linhaEncoder = linha.getEncoder();
    RelativeEncoder intakeEncoder = intake.getEncoder();

    static DigitalInput TemAlga = new DigitalInput(0);

    

    PIDController pidi = new PIDController(0, 0, 0);
    private static final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);

    public static void linhaset(double velocidade) {
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

    public static boolean TemAlga(){
        return TemAlga.get();
    }

    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
