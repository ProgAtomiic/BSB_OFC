// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDSubsystem extends SubsystemBase {
  double Correcao;

  public PIDSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double PID(double Kp, double Ki, double Kd, double Alvo, double LeituraAtual) {
    try (PIDController PID = new PIDController(0, 0, 0)) {
      Correcao = PID.calculate(LeituraAtual, Alvo);
    }
    return Correcao;
  }

  public double PIDF(double Kp, double Ki, double Kd, double Ks, double Kv, double Ka, double Alvo, double LeituraAtual,
      double VelMotor) {
    SimpleMotorFeedforward Feedforward = new SimpleMotorFeedforward(Ks, Kv, Ka);
    try (PIDController PID = new PIDController(Kp, Ki, Kd)) {
      Correcao = (PID.calculate(LeituraAtual, Alvo) + Feedforward.calculate(VelMotor));
    }
    return Correcao;
  }
}
