package frc.robot.vision;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class IndicatorLights {
    //TODO: config constants need to be made for these params
    private final AddressableLED led = new AddressableLED(20); //dummy port
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(70);

    public IndicatorLights(){
        led.setLength(70);
        led.setData(ledBuffer);
        led.start();
    }

    public AddressableLEDBuffer getLedBuffer(){
        return ledBuffer;
    }

    public AddressableLED getLED(){
        return led;
    }
    public void setBlue(){
        for(int i = 0; i < ledBuffer.getLength(); i++){
            ledBuffer.setRGB(i,0,0,255);
        }
    }

    public void setRed(){
        for(int i = 0; i < ledBuffer.getLength(); i++){
            ledBuffer.setRGB(i,255,0,0);
        }
    }

    public void setPurple(){
        for(int i = 0; i < ledBuffer.getLength(); i++){
            ledBuffer.setRGB(i,128,0,128);
        }
    }

    public void setGreen(){
        for(int i = 0; i < ledBuffer.getLength(); i++){
            ledBuffer.setRGB(i,0,255,0);
        }
    }

//    public void setRainbow(){
//        for(int i = 0; i < ledBuffer.getLength(); i++){
//        }
//    }

    public void setOff(){
        for(int i = 0; i < ledBuffer.getLength(); i++){
            ledBuffer.setRGB(i,0,0,0);
        }
    }
}
