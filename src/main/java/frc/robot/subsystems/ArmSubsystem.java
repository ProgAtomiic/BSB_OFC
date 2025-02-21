// package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// ////////////////////
// ///
// public class ArmSubsystem extends SubsystemBase {

//     public int angulo_alvo = 0;

//     static SparkMax arm = new SparkMax(14, SparkMax.MotorType.kBrushless);
//     static RelativeEncoder encoder = arm.getEncoder();

//     static Servo ServoMotor = new Servo(1);

//     PIDController pidi = new PIDController(0.04, 0.00, 0.00005);
//     // PIDController pidi = new PIDController(0.04, 0.0001, 0.000005);
//     private final ArmFeedforward feedforward = new ArmFeedforward(0.3, 0.7, 0.45);

//     public static void reset_motor() {
//         encoder.setPosition(0);
//     }

//     public void angleset(double angulo) {

//             double vel_max = 1.5;
//             double velocidade = pidi.calculate(angleget(), angulo);
//             double velocidade1 = feedforward.calculate(Math.toRadians(angleget()), velocidade);
//             double soma_velocidade = velocidade + velocidade1;

//             if (Math.abs(soma_velocidade) < vel_max) {
//                 arm.setVoltage(soma_velocidade);
//             } else {
//                 arm.setVoltage(vel_max * (soma_velocidade / Math.abs(soma_velocidade)));
//             }
//             System.out.print("velocidade");
//             System.out.println(soma_velocidade);
        

//         // if (ElevatorSubsystem.GetPosicaoElevador() >= 50) {// EM ROTACOES
//         //     double vel_max = 1.5;
//         //     double velocidade = pidi.calculate(angleget(), angulo);
//         //     double velocidade1 = feedforward.calculate(Math.toRadians(angleget()), velocidade);
//         //     double soma_velocidade = velocidade + velocidade1;

//         //     if (Math.abs(soma_velocidade) < vel_max) {
//         //         arm.setVoltage(soma_velocidade);
//         //     } else {
//         //         arm.setVoltage(vel_max * (soma_velocidade / Math.abs(soma_velocidade)));
//         //     }
//         //     System.out.print("velocidade");
//         //     System.out.println(soma_velocidade);
//         // } else {
//         //     System.err.println("Baixo demais");
//         // }

//     }

//     public static void set(double velocidade) {
//         arm.set(velocidade);
//     }

//     public double angleget() {
//         return (encoder.getPosition() * 14.4);
//     }

//     public void stoparm() {
//         arm.set(0);
//     }

//     public static void LigarServo(double Velocidade) {
//         ServoMotor.set(Velocidade);
//     }

//     public static void DesligarServo() {
//         ServoMotor.set(0);
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

//     /*
//      * public void angle1(){ //COMPLETAMENTE ABAIXADA
//      * //MOVE MOTOR ARM PARA O ANGULO DA GARRA COM O SENSOR DE VALOR ABSOLUTO DE
//      * MOTOR
//      * }
//      * public void angle2(){ //ANGULADA PARA CIMA 1
//      * //MOVE MOTOR ARM PARA O ANGULO DA GARRA COM O SENSOR DE VALOR ABSOLUTO DE
//      * MOTOR
//      * }
//      * public void angle3(){ //ANGULADA PARA CIMA 2
//      * //MOVE MOTOR ARM PARA O ANGULO DA GARRA COM O SENSOR DE VALOR ABSOLUTO DE
//      * MOTOR
//      * }
//      */

// }
