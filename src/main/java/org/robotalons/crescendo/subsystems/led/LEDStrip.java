// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.led;

import org.robotalons.crescendo.subsystems.superstructure.SuperstructureSubsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDStrip extends SubsystemBase {
  public static AddressableLED M_LED;
  public static AddressableLEDBuffer M_LEDBUFFER;

  /** Creates a new LEDStrip. */
  public LEDStrip(int LENGTH, int PORT) {
    M_LED = new AddressableLED(9);
    M_LEDBUFFER = new AddressableLEDBuffer(60);
    M_LED.setLength(M_LEDBUFFER.getLength());
    M_LED.setData(M_LEDBUFFER);
    M_LED.start();
  }

  public static synchronized void setGreen(){
    for (int i = 0; i < M_LEDBUFFER.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      M_LEDBUFFER.setHSV(i, 115, 100, 70);
   }
   
   M_LED.setData(M_LEDBUFFER);
  }

  public static synchronized void setBlack(){
    for (int i = 0; i < M_LEDBUFFER.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      M_LEDBUFFER.setHSV(i, 0, 0, 0);    
    }
    M_LED.setData(M_LEDBUFFER);
  }

  public static synchronized void blinkGreen(){
    setGreen();
    new WaitCommand(0.5);
    setBlack();
  }

  public static synchronized void setBlue(){
    for (int i = 0; i < M_LEDBUFFER.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      M_LEDBUFFER.setHSV(i, 235, 100, 75);
   }
   
   M_LED.setData(M_LEDBUFFER);
  }

  public static synchronized void setPurple(){
    for (int i = 0; i < M_LEDBUFFER.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      M_LEDBUFFER.setHSV(i, 265, 100, 85);
   }
   
   M_LED.setData(M_LEDBUFFER);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    while(SuperstructureSubsystem.getNoteDetector().get()){
      blinkGreen();
    }
  }
}
