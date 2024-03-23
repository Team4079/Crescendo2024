package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;

/**
 * The {@link LED} class includes all the methods to control the LEDs.
 */
public class LED extends SubsystemBase {
  private AddressableLED alignmentIndication1;
  // private AddressableLED alignmentIndication2;

  private AddressableLEDBuffer ledBuffer1;
  // private AddressableLEDBuffer ledBuffer2;

  public LED() {
    alignmentIndication1 = new AddressableLED(5); // 6 and 7

    ledBuffer1 = new AddressableLEDBuffer(21);

    alignmentIndication1.setLength(ledBuffer1.getLength());

    alignmentIndication1.setData(ledBuffer1);

    alignmentIndication1.start();
  }

  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      rainbowRGB(SwerveGlobalValues.hightideLED[0], SwerveGlobalValues.hightideLED[1],
          SwerveGlobalValues.hightideLED[2]);
      // setRedColor();
    }
  }

  /**
   * Sets the color for each of the LEDs based on HSV values
   * 
   * @param h (Hue) Integer values between 0 - 180
   * @param s (Saturation) Integer values between 0 - 255
   * @param v (Value) Integer values between 0 - 255
   * @return void
   */
  public void rainbowHSV(int H, int S, int V) {
    for (int i = 0; i < ledBuffer1.getLength(); i++) {
      ledBuffer1.setHSV(i, H, S, V);
    }
    alignmentIndication1.setData(ledBuffer1);
  }

  /**
   * Sets the color for each of the LEDs based on RGB values
   * 
   * @param r (Red) Integer values between 0 - 255
   * @param g (Green) Integer values between 0 - 255
   * @param b (Blue) Integer values between 0 - 255
   * @return void
   */
  public void rainbowRGB(int R, int G, int B) {
    for (int i = 0; i < ledBuffer1.getLength(); i++) {
      ledBuffer1.setRGB(i, R, G, B);
    }
    alignmentIndication1.setData(ledBuffer1);
  }

  public void setRedColor() {
    for (int i = 0; i < ledBuffer1.getLength(); i++) {
      ledBuffer1.setRGB(i, 255, 0, 0);
    }
    alignmentIndication1.setData(ledBuffer1);
  }
}