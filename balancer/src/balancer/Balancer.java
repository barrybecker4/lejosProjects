
package balancer;

import balancer.segway.EV3Segway;
import balancer.segway.GyroSensor;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.HiTechnicGyro;
import lejos.utility.Delay;

/**
 * A Balancing Robot.
 * Makes use of the EV3Segway classes by Brian Bagnell to do balancing and movement.
 * 
 * @author Barry Becker
 */
public class Balancer {

    /** diameter of the EV3 wheels */
    private static final double MEDIUM_WHEEL_DIAMETER = 5.2;
    
    /** diameter of large Mindstorm wheels */
    private static final double LARGE_WHEEL_DIAMETER = 7.2;
    
    public static void main(String[] args) throws Exception {
        LCD.drawString("Balancer Robot", 1, 3);
        Delay.msDelay(1000);
        LCD.clear();
        
        System.out.println("Lay robot down");
        System.out.println("to calibrate");
        System.out.println("the gyro");
        
        NXTMotor left = new NXTMotor(MotorPort.B);
        NXTMotor right = new NXTMotor(MotorPort.C);
        //EncoderMotor right = new EV3LargeRegulatedMotor(MotorPort.C);
        GyroSensor gyro = new GyroSensor(new HiTechnicGyro(SensorPort.S1));
        //EV3IRSensor irSensor = new EV3IRSensor(SensorPort.S4);
        //SampleProvider distanceSampler = irSensor.getDistanceMode();
        
        EV3Segway robot = new EV3Segway(left, right, gyro, MEDIUM_WHEEL_DIAMETER);
        
        /*
        Delay.msDelay(2000);
        robot.wheelDriver(-120, -120); // Move forward
        
        float dist[] = new float[1];
        while (!Button.ESCAPE.isDown()) {
            distanceSampler.fetchSample(dist, 0);
            //System.out.println("dist="+dist[0]);
            if (dist[0] < 0.2) {
                robot.wheelDriver(70, 35); // Arc back
                Delay.msDelay(3000);
                robot.wheelDriver(-120, -120); // Move forward
            }
            Delay.msDelay(350);
        }
        
        Button.waitForAnyPress();
        irSensor.close();
        gyro.close();
        left.close();
        right.close();
        Delay.msDelay(1000);*/
        
        /*
        float[] rate = new float[1];
        LCD.drawString("Gyro: ", 2, 1);
        while (!Button.ESCAPE.isDown()) {
            gyro.fetchSample(rate, 0);
            float gyroRaw = rate[0];
            //System.out.println("gyroRaw="+gyroRaw);
            LCD.drawString(""+gyroRaw, 12, 1);
            Delay.msDelay(100);
        }*/
        Button.ENTER.waitForPressAndRelease();
        System.exit(0);
    }
}
