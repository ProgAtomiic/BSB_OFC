// package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;

// ////////////////////
// /// 
// public class ArmSubsystem {
//     SparkMax arm = new SparkMax(13, SparkMax.MotorType.kBrushless);
//     RelativeEncoder encoder = arm.getEncoder();
//     DutyCycleEncoder absencoder = new DutyCycleEncoder(9);

//     PIDController pidi = new PIDController(0, 0, 0);
//     private final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);
//     double angulo = absencoder.get() * 360;// CONFERIR CONVERSÃO PARA GRAUS

//     public void angleset(double angulo, double velocidade) {
//         arm.set(feedforward.calculate(angulo, velocidade));
//     }

//     public void setpoint() {
//         arm.set(feedforward.calculate(0, 0.2));
//     }

//     public void stoparm() {
//         arm.set(0);
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
