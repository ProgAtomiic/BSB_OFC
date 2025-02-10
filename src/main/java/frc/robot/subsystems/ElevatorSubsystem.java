// package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class ElevatorSubsystem extends SubsystemBase {
//     static DigitalInput fimdecurso1 = new DigitalInput(0); // BAIXO INTERNO
//     static DigitalInput fimdecurso2 = new DigitalInput(1); // CIMA INTERNO
//     static DigitalInput fimdecurso3 = new DigitalInput(2); // BAIXO EXTERNO
//     static DigitalInput fimdecurso4 = new DigitalInput(3); // CIMA EXTERNO

//     static SparkMax elevadorext = new SparkMax(11, MotorType.kBrushless);
//     static SparkMax elevadorint = new SparkMax(9, MotorType.kBrushless);
//     static RelativeEncoder encoderint = elevadorext.getEncoder();
//     static RelativeEncoder encoderext = elevadorint.getEncoder();
//     static DutyCycleEncoder absencoderint = new DutyCycleEncoder(8);
//     static DutyCycleEncoder absencoderext = new DutyCycleEncoder(7);

//     static double anguloint = absencoderint.get() * 360;
//     static double anguloext = absencoderext.get() * 360;

//     PIDController pidi = new PIDController(0, 0, 0);
//     private static final ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0, 0);

//     private double pidvoltage = 0;
//     private double feedforwardVoltage = 0;

//     public ElevatorSubsystem() {
//     }

//     // FAZER FUNÇÃO DE RODAR POR GRAUS (FAZER COM UM WHILE)

//     public static void ligarelevador(double velinterno, double velexterno) {
//         elevadorint.set(feedforward.calculate(velinterno));
//         elevadorext.set(feedforward.calculate(velxterno));
//     }

//     public static void ligaelevador_externo(double velocidade) {
//         elevadorext.set(feedforward.calculate(velocidade));
//     }

//     public static double angulo_elevador_interno() {
//         return anguloint;
//     }

//     public static double angulo_elevador_externo() {
//         return anguloext;
//     }

//     public static void intakepeca() {// FAZER POR GRAUS
//         //
//         elevadorint.set(feedforward.calculate(0));
//         //
//     }

//     public static void setpoint() {
//         if (fimdecurso1.get() == false) {
//             elevadorint.set(-0.2);
//         }
//         if (fimdecurso3.get() == false) {
//             elevadorext.set(-0.2);
//         }
//     }

//     public static void settop_interno() {
//         if (fimdecurso2.get() == false) {
//             elevadorint.set(0.2);
//         }
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
//      * public void desce_limite_interno(){
//      * while(fimdecurso1.get()==false){
//      * elevadorint.set(-0.2); //DESCE O ELEVADOR INTERNO ATÉ O LIMITE DO FIM DE
//      * CURSO
//      * }
//      * }
//      * public void sobe_limite_interno(){
//      * while(fimdecurso2.get()==false){
//      * elevadorint.set(0.2); //SOBE O ELEVADOR INTERNO ATÉ O LIMITE DO FIM DE CURSO
//      * }
//      * }
//      * public void desce_limite_externo(){
//      * while(fimdecurso3.get()==false){
//      * elevadorext.set(-0.2); //DESCE O ELEVADOR INTERNO ATÉ O LIMITE DO FIM DE
//      * CURSO
//      * }
//      * }
//      * public void sobe_limite_externo(){
//      * while(fimdecurso4.get()==false){
//      * elevadorext.set(0.2); //SOBE O ELEVADOR INTERNO ATÉ O LIMITE DO FIM DE CURSO
//      * }
//      * }
//      */
// }