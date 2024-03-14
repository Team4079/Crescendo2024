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
  // private AddressableLED alignmentIndication1;
  private AddressableLED alignmentIndication2;

  // private AddressableLEDBuffer ledBuffer1;
  private AddressableLEDBuffer ledBuffer2;

  public LED() {
    // alignmentIndication1 = new AddressableLED(6); // 6 and 7
    alignmentIndication2 = new AddressableLED(6);

    // ledBuffer1 = new AddressableLEDBuffer(21);
    ledBuffer2 = new AddressableLEDBuffer(21);

    // alignmentIndication1.setLength(ledBuffer1.getLength());
    alignmentIndication2.setLength(ledBuffer2.getLength());

    // alignmentIndication1.setData(ledBuffer1);
    alignmentIndication2.setData(ledBuffer2);

    // alignmentIndication1.start();
    alignmentIndication2.start();
  }

  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      rainbow(SwerveGlobalValues.hightideLED[0], SwerveGlobalValues.hightideLED[1], SwerveGlobalValues.hightideLED[2]);
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
  public void rainbow(int h, int s, int v) {
    // rainbowOn = true;
    for (int i = 0; i < ledBuffer2.getLength(); i++) {
      // ledBuffer1.setHSV(i, h, s, v);
      ledBuffer2.setRGB(i, h, s, v);
    }
    // alignmentIndication1.setData(ledBuffer1);
    alignmentIndication2.setData(ledBuffer2);
  }
}