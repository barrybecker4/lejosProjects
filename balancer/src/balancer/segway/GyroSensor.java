package balancer.segway;
import lejos.hardware.sensor.HiTechnicGyro;

/**
 * <p>This class encapsulates the HiTechnicGyro and provides a more convenient API .</p> 
 * 
 * @author Barry Becker
 */
public class GyroSensor {
  
    private static final int NUM_CALIBRATON_SAMPLES = 50;
    
    /**
     * Gyro offset control
     * The gyro sensor will drift with time.  This constant is used in a simple long term averaging
     * to adjust for this drift. Every time through the loop, the current gyro sensor value is
     * averaged into the gyro offset weighted according to this constant.
     */
    private static final double EMAOFFSET = 0.0004;  // originally 0.0005
    
    private HiTechnicGyro gyro;
    
    /** initial baseline value */
    private float gyroBaseline = 0;
    
    /** calibration offset */
    private double gOffset;
    
    //private double gAngleGlobal = 0;

    private double gyroSpeed;
    private double gyroAngle = 0; 

    
    /**
     * The gyro sensor should be perfectly still during construction 
     * so it could be accurately calibrated.
     * @param gyro the HiTechniqueGyro instance.
     */
    public GyroSensor(HiTechnicGyro gyro) {
    
        this.gyro = gyro;
        // initial calibration
        gyroBaseline = getGyroBaseline();
    }
    
    /**
     * @rturn Integrated angle in degrees
     */
    public double getAngle() {
        return gyroAngle;
    }
    
    /**
     * @return angular velocity in degrees per second.
     */
    public double getAngularVelocity() {
        return gyroSpeed;
    }
    
    /**
     * Get the data from the gyro. 
     * Updates the internal gyroSpeed and gyroAngle based on updated information from the Gyro Sensor.
     * Maintains an automatically adjusted gyro offset as well as the integrated gyro angle.
     * @param elapsedTime 
     */
    public void updateGyroData(double elapsedTime) {
        
        float gyroRaw = getRawAngularVelocity();
        gOffset = EMAOFFSET * gyroRaw + (1-EMAOFFSET) * gOffset;
        gyroSpeed = gyroRaw - gOffset; // Angular velocity (degrees/sec)
        gyroAngle += gyroSpeed * elapsedTime; // Absolute angle (degrees)
    }
    
    /**
     * @return the calibrated angular velocity from the gyroscope sensor.
     */
    public float getRawAngularVelocity() {
    
        float[] rate = new float[1];
        gyro.fetchSample(rate, 0);
        float rotSpeed = rate[0] - gyroBaseline;
        return rotSpeed; 
    }
    
    /**
     * This function returns a suitable initial gyro offset.  It takes
     * N gyro samples and averages them to get the offset.
     * It also check the max and min during that time and if
     * the difference is larger than one it rejects the data and
     * gets another set of samples.
     */
    private float getGyroBaseline() {
        
        // read a few values and take the average
        float total = 0;
       
        for (int i=0; i<NUM_CALIBRATON_SAMPLES; i++) {
            total += getRawAngularVelocity();
            ThreadUtil.sleep(5);
        }
       
        float gyroBaseline = total / NUM_CALIBRATON_SAMPLES;
        return gyroBaseline;
    }
}