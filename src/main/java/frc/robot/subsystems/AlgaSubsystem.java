// package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class AlgaSubsystem extends SubsystemBase {
//     static SparkMax linha = new SparkMax(7, SparkMax.MotorType.kBrushless);
//     static SparkMax intake = new SparkMax(0, SparkMax.MotorType.kBrushless);

//     RelativeEncoder linhaEncoder = linha.getEncoder();
//     RelativeEncoder intakeEncoder = intake.getEncoder();
//     DutyCycleEncoder abslinha = new DutyCycleEncoder(7);
//     DutyCycleEncoder absintake = new DutyCycleEncoder(6);

//     PIDController pidi = new PIDController(0, 0, 0);
//     private static final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);

//     double linhaAngulo = abslinha.get() * 360;
//     double intakeAngulo = absintake.get() * 360;

//     public static void linhaset(double angulo, double velocidade) {
//         linha.set(feedforward.calculate(angulo, velocidade));
//     }

//     public static void ligaintake(double velocidade) {
//         intake.set(velocidade);
//     }

//     public static void linhasetpoint() {
//         linha.set(feedforward.calculate(0, 0.2));
//     }

//     public static void desligaintake() {
//         intake.set(0);
//     }

//     public static void desligalinha() {
//         linha.set(0);
//     }

//     public boolean exampleCondition() {
//         // Query some boolean state, such as a digital sensor.
//         return false;
//     }

//     public void periodic() {
//         // This method will be called once per scheduler run
//     }

//     public void simulationPeriodic() {
//         // This method will be called once per scheduler run during simulation
//     }

// }
