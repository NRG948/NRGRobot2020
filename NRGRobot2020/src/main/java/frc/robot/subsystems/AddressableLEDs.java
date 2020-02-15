package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem that control addressable leds
 */
public class AddressableLEDs extends SubsystemBase {
  private static final AddressableLED led = new AddressableLED(8);
  private static final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(12);

  public AddressableLEDs(){
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
  }

  /**
   * Sets the entire led buffer to a color.
   */
  public static void setAll(Color8Bit color){
    for (int i = 0; i < ledBuffer.getLength(); ++i) {
      ledBuffer.setLED(i, color);
    }
  }

  /**
   * Sets the led buffer at index i to the given color.
   */
  public static void set(int i, Color8Bit color){
    ledBuffer.setLED(i, color);
  }

  /**
   * Sends the buffer color data to the led lights.
   */
  public static void sendToLeds(){
    led.setData(ledBuffer);
    led.start();
  }
}