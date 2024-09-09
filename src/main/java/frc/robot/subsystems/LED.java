package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The LED subsystem controls the LED strip on the robot.
 * It uses an AddressableLED and AddressableLEDBuffer to manage the LED colors and patterns.
 */
public class LED extends SubsystemBase {
  private final AddressableLED alignmentIndication1;
  private final AddressableLEDBuffer addressableLEDBuffer;

  /**
   * Constructs a new LED subsystem.
   * Initializes the AddressableLED and AddressableLEDBuffer with a specified length.
   */
  public LED() {
    alignmentIndication1 = new AddressableLED(9);
    addressableLEDBuffer = new AddressableLEDBuffer(55);
    alignmentIndication1.setLength(addressableLEDBuffer.getLength());
    alignmentIndication1.setData(addressableLEDBuffer);
    alignmentIndication1.start();
  }

  /**
   * This method will be called once per scheduler run.
   * Updates the LED pattern based on the robot state.
   */
  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      highTideFlow();
    } else {
      setRGB(0, 0, 0);
    }
  }

  /**
   * Sets the color for each of the LEDs based on RGB values.
   *
   * @param r (Red) Integer values between 0 - 255
   * @param g (Green) Integer values between 0 - 255
   * @param b (Blue) Integer values between 0 - 255
   */
  public void setRGB(int R, int G, int B) {
    for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
      addressableLEDBuffer.setRGB(i, R, G, B);
    }
    alignmentIndication1.setData(addressableLEDBuffer);
  }

  /**
   * Sets the LED color to tan.
   */
  public void setTanColor() {
    setRGB(255, 120, 20);
  }

  /**
   * Sets the LED color to red.
   */
  public void setRedColor() {
    setRGB(255, 0, 0);
  }

  /**
   * Sets the LED color to green.
   */
  public void setGreenColor() {
    setRGB(0, 255, 0);
  }

  /**
   * Sets the LED color to high tide (a specific shade of blue-green).
   */
  public void setHighTide() {
    setRGB(0, 182, 174);
  }

  /**
   * Creates a flowing high tide effect on the LED strip.
   * The effect is based on a sine wave pattern that changes over time.
   */
  public void highTideFlow() {
    long currentTime = System.currentTimeMillis();
    int length = addressableLEDBuffer.getLength();

    final int waveSpeed = 30;
    final int waveWidth = 55;

    for (int i = 0; i < length; i++) {
      double wave = Math.sin((i + ((double)currentTime / waveSpeed)) % length * (2 * Math.PI / waveWidth));

      wave = (wave + 1) / 2;

      int r = (int)(wave * 0);
      int g = (int)(wave * 200);
      int b = (int)(wave * 50);

      addressableLEDBuffer.setRGB(i, r, g, b);
    }
    alignmentIndication1.setData(addressableLEDBuffer);
  }
}