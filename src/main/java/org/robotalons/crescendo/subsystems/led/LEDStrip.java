// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package org.robotalons.crescendo.subsystems.led;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicInteger;

public final class LEDStrip{

  public static int M_LENGTH;
  public static AddressableLED M_LED;
  public static AddressableLEDBuffer M_BUFFER;
  public static boolean shouldAnimate;
  public static ExecutorService executor = Executors.newFixedThreadPool(1);

  /**
   * Creates LEDStrip 
   * @param LENGTH
   * @param PORT
   */
  public LEDStrip(int LENGTH, int PORT){
    // Configure LED strip and Buffer for coding
    M_LENGTH = LENGTH;
    M_LED = new AddressableLED(PORT);
    M_BUFFER = new AddressableLEDBuffer(LENGTH);
    M_LED.setLength(M_BUFFER.getLength());
    M_LED.setData(M_BUFFER);
    M_LED.start();
  }

  public static List<LEDIdentifier> ALL_COLORS_IDENTIFIERS = List.of(
    LEDIdentifier.DARK_GREEN,
    LEDIdentifier.WHITE,
    LEDIdentifier.DARK_BLUE,
    LEDIdentifier.DARK_PURPLE,
    LEDIdentifier.DARK_RED,
    LEDIdentifier.CYAN,
    LEDIdentifier.YELLOW,
    LEDIdentifier.GRAY,
    LEDIdentifier.BLACK);

  public enum LEDIdentifier {

    DARK_GREEN((140), (80), (80)),
    WHITE((0),(0),(100)),
    DARK_BLUE((250),(100),(100)),
    DARK_PURPLE((270),(80),(90)),
    DARK_RED((0), (100), (100)),
    CYAN((184), (96), (87)),
    YELLOW((53), (100), (100)),
    GRAY((0), (0), (32)),
    BLACK((0), (0), (0));

    public final int Hue, Saturation, Value;

    LEDIdentifier(final int Hue, final int Saturation, final int Value) {
      this.Hue = Hue;
      this.Saturation = Saturation;
      this.Value = Value;
    }
  /**
    * @return int representing Hue Value
    */
    public int getHue() {
      return this.Hue;
    }
  /**
    * @return int representing Saturation Value
    */
    public int getSaturation() {
      return this.Saturation;
    }
  /**
    * @return int representing Value Value :)
    */
    public int getValue() {
      return this.Value;
    }
  }

  /**
    * Sets entire LED Strip to certain color
    * @param COLOR representing the color the entire LED Strip is set to
    */
  public void setColor(LEDIdentifier COLOR){
    for (int i = 0; i < M_BUFFER.getLength(); i++) {
      M_BUFFER.setHSV(i, COLOR.getHue(), COLOR.getSaturation(), COLOR.getValue());
    }
    M_LED.setData(M_BUFFER);
  }

  /**
   * @param COLOR representing the color the entire LED Strip is set to
   * @param INDEX represents certain LED in Strip
   */
  public void setColor(LEDIdentifier COLOR, int INDEX){
    M_BUFFER.setHSV(INDEX, COLOR.getHue(), COLOR.getSaturation(), COLOR.getValue());
    M_LED.setData(M_BUFFER);
  }

  //TODO: Create bounce method
  /**
   * Creates a bounce animation with a color block of length
   * @param COLOR representing the color the entire LED Strip is set to
   * @param BACKGROUND representing the other color that will be between LEDSs
   * @param LENGTH how long the block that bounces is
   */
  public void setBounce(LEDIdentifier BOUNCE, LEDIdentifier BACKGROUND, int LENGTH){}

  /**
   * Creates a flashing animation with a led strip
   * @param COLOR1 representing the color the entire LED Strip is set to, will be the starting color
   * @param COLOR2 representing the other color that will be between LEDSs, will be the color afterwards
   * @param TIME how long each color is set before switched in seconds
   */
  public void setFlash(LEDIdentifier COLOR1, LEDIdentifier COLOR2, int TIME){
    setColor(COLOR1);
    executor.submit(() -> {
      try {
        Thread.sleep((long)(TIME * 1000.0));
        setColor(COLOR2);
      }
      catch(final InterruptedException ignore){}
    });
  }

  /**
   * Creates a pattern in the LED strip that runs
   * @param HOPPER representing the other color that will hop through the strip
   * @param BACKGROUND representing the other color that will flash
   * @param TIME how long each set of color lasts in seconds
   */
  public void setHop(LEDIdentifier HOPPER, LEDIdentifier BACKGROUND, double TIME){
    setColor(BACKGROUND);
    final AtomicInteger i = new AtomicInteger(0);
    for(i.get(); i.get() < M_BUFFER.getLength(); i.incrementAndGet()){
      executor.submit(() -> {
      try {
        Thread.sleep((long)(TIME * 1000.0));
        setColor(HOPPER, i.get());
        if(i.get() - 1 != 0){
          setColor(BACKGROUND, i.get() - 1);
        }
      }
      catch(final InterruptedException ignore){}
    });
    }
  }

  // @Override
  public void periodic() {
  }
}