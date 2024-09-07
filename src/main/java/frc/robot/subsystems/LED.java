package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The {@link LED} class includes all the methods to control the LEDs.
 */
public class LED extends SubsystemBase {
  private AddressableLED alignmentIndication1;

  private AddressableLEDBuffer shawnComesIntoMyPEClass;

  public LED() {
    alignmentIndication1 = new AddressableLED(9); // 6 and 7

    shawnComesIntoMyPEClass = new AddressableLEDBuffer(55);

    alignmentIndication1.setLength(shawnComesIntoMyPEClass.getLength());

    alignmentIndication1.setData(shawnComesIntoMyPEClass);

    alignmentIndication1.start();
  }

  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      highTideFlow();
    }
    else {
      rainbowRGB(0, 0, 0);
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
    for (int i = 0; i < shawnComesIntoMyPEClass.getLength(); i++) {
      shawnComesIntoMyPEClass.setHSV(i, H, S, V);
    }
    alignmentIndication1.setData(shawnComesIntoMyPEClass);
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
    for (int i = 0; i < shawnComesIntoMyPEClass.getLength(); i++) {
      shawnComesIntoMyPEClass.setRGB(i, R, G, B);
    }
    alignmentIndication1.setData(shawnComesIntoMyPEClass);
  }

  public void setTanColor() {
    for (int i = 0; i < shawnComesIntoMyPEClass.getLength(); i++) {
      shawnComesIntoMyPEClass.setRGB(i, 255, 120, 20);
    }
    alignmentIndication1.setData(shawnComesIntoMyPEClass);
  }
  
  public void setRedColor() {
    for (int i = 0; i < shawnComesIntoMyPEClass.getLength(); i++) {
      shawnComesIntoMyPEClass.setRGB(i, 255, 0, 0);
    }
    alignmentIndication1.setData(shawnComesIntoMyPEClass);
  }

  public void setGreenColor() {
    for (int i = 0; i < shawnComesIntoMyPEClass.getLength(); i++) {
      shawnComesIntoMyPEClass.setRGB(i, 0, 255, 0);
    }
    alignmentIndication1.setData(shawnComesIntoMyPEClass);
  }

  public void setPurpleColor() {
    for (int i = 0; i < shawnComesIntoMyPEClass.getLength(); i++) {
      shawnComesIntoMyPEClass.setRGB(i, 160, 32, 240);
    }
    alignmentIndication1.setData(shawnComesIntoMyPEClass);
  }

  public void setHighTide() {
    for (int i = 0; i < shawnComesIntoMyPEClass.getLength(); i++) {
      shawnComesIntoMyPEClass.setRGB(i, 0, 182, 174);
    }
    alignmentIndication1.setData(shawnComesIntoMyPEClass);
  }

  public void highTideFlow() {
    long currentTime = System.currentTimeMillis();
    int length = shawnComesIntoMyPEClass.getLength();

    final int waveSpeed = 30; 
    final int waveWidth = 55;

    for (int i = 0; i < length; i++) {
      double wave = Math.sin((i + ((double)currentTime / waveSpeed)) % length * (2 * Math.PI / waveWidth));

      wave = (wave + 1) / 2;

      int r = (int)(wave * 0); 
      int g = (int)(wave * 200);
      int b = (int)(wave * 50);

      shawnComesIntoMyPEClass.setRGB(i, r, g, b);
    }
    alignmentIndication1.setData(shawnComesIntoMyPEClass);
}

}