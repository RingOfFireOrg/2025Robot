package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;
  private int ledSegment = 20;


  public LEDSubsystem() {
    m_led = new AddressableLED(0); 
    m_ledBuffer = new AddressableLEDBuffer(ledSegment);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    m_led.setData(m_ledBuffer);
  }

  // --------------------------------- SET RGB FUNCTIONS ------------------------------------- \\

  public void setLEDRGB(int red, int green, int blue) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {m_ledBuffer.setRGB(i, red, green, blue);}    
    m_led.setData(m_ledBuffer);
  }

  public void setLEDHSV(int h, int s, int v) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {m_ledBuffer.setHSV(i, h, s, v);}    
    m_led.setData(m_ledBuffer);
  }

  /*  -------------------------------- Blinking Pattern --------------------------------------- */   
    



  // ------------------------------------ Red chase ------------------------------------------------- \\                        

  public void redMove() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 255 / 30)) % 255;
      m_ledBuffer.setRGB(i, 255-hue, 0, 0);
    }
    m_rainbowFirstPixelHue += 6;
    m_rainbowFirstPixelHue %= 255;
    m_led.setData(m_ledBuffer);
  } 

  public void redMove(double speed) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 255 / 30)) % 255;
      m_ledBuffer.setRGB(i, 255-hue, 0, 0);
    }
    m_rainbowFirstPixelHue += speed/1000;
    m_rainbowFirstPixelHue %= 255;
    m_led.setData(m_ledBuffer);
  } 




/*  --------------------------------Shifting orange Pattern for Red Alliance--------------------------------------- */   
  int firstOrange = 2;
  boolean flip = false;

  public void shiftingOrange() {
    if (flip == false) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, firstOrange, 0);
      }
      firstOrange += 1;
      m_led.setData(m_ledBuffer);
      if (firstOrange >= 140) {
        flip = true;
      }
    }
    if (flip == true) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, firstOrange, 0);
      }
      firstOrange -= 1;
      m_led.setData(m_ledBuffer);
      if (firstOrange <= 20) {
        flip = false;
      }
    }
  }


 


  /*  --------------------------------Shifting orange Pattern for Blue Alliance--------------------------------------- */   
  
  public void shiftingBlue() {}

  public void shiftingBlue_BAR() {}


  /*  ------------------------------------- Random --------------------------------------------- */   



  public void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    m_rainbowFirstPixelHue += 3;
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
  }
  
    
  public void blueGradient() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 128);
      m_ledBuffer.setRGB(i, 20, hue, 255);
    }
    m_rainbowFirstPixelHue += 3;
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);

  }

}