// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class LevelSet {
  public String LadoEscolhido;
  public int Level;
  public double Contador = 0;
  public boolean Vibrando;

  public LevelSet() {
  }

  public double Contador() {
    if(LadoEscolhido.isEmpty() == false){
      return Contador++;
    } else{
      return Contador = 0;
    }

  }

  public void RumbleControle(XboxController Controle, RumbleType rumbleType, double Timer){

    if (((int) (Timer / 5)) % 2 == 0 && !Vibrando) {
      Controle.setRumble(rumbleType, 1);
      Vibrando = true; 
      
  } else if (((int) (Timer / 5)) % 2 == 1 && Vibrando) {
    Controle.setRumble(rumbleType, 0);
      Vibrando = false; 
  }

  }

  public void ResetContador() {
    Contador = 0;

  }

  public void SetLado(String lado) {
    LadoEscolhido = lado;
  }

  public String GetLado() {
    return LadoEscolhido;
  }
  
  

  public void SetLevel(int newLevel) {
    if (LadoEscolhido.isEmpty()) {

      System.out.println("Escolha um lado primeiro!");
      return; // Não permite escolher nível sem lado
    } else {
      Level = newLevel;
    }
  }

  public boolean TemLevel() {
    return Level > 0; // Retorna true se Level for maior que 0 (um nível foi escolhido)
  }

  public int GetLevel() {
    return Level; // Retorna true se Level for maior que 0 (um nível foi escolhido)
  }

}
