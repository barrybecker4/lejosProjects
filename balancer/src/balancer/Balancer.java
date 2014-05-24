
package balancer;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.HiTechnicGyro;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Balancer {

	/** diameter of the EV3 wheels */
    private static final double WHEEL_DIAMETER = 5.2;
    
    public static void main(String[] args) throws Exception {
        LCD.drawString("Segway Balancer", 1, 3);
        Delay.msDelay(1000);
        LCD.clear();
        
        NXTMotor left = new NXTMotor(MotorPort.B);
        NXTMotor right = new NXTMotor(MotorPort.C);
        //EncoderMotor right = new EV3LargeRegulatedMotor(MotorPort.C);
        HiTechnicGyro gyro = new HiTechnicGyro(SensorPort.S1);
        //EV3IRSensor irSensor = new EV3IRSensor(SensorPort.S4);
        //SampleProvider distanceSampler = irSensor.getDistanceMode();
        
        EV3Segoway robot = new EV3Segoway(left, right, gyro, WHEEL_DIAMETER); //EV3SegowayPilot.WHEEL_SIZE_NXT2);
        
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
