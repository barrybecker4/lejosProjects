
package helloEv3;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.lcd.LCD;
//import lejos.nxt.*
import lejos.utility.Delay;

public class HelloWorld {

    public static void main(String[] args) {
        LCD.drawString("Plugin Test", 1, 3);
        Delay.msDelay(2000);
        System.out.println("Hello Barry!");
        for (int i=0; i<4; i++) {
            Sound.buzz();
            Sound.buzz();
            Sound.playTone(1000, 100);
        }
        
        Button.waitForAnyPress();
                
        Delay.msDelay(1000);
        System.exit(0);
    }
}
