package balancer.segway;

import lejos.robotics.EncoderMotor;

/**
 * <p>Controls the two wheels and maintains their state. 
 * Refer to http://www.hitechnic.com/blog/gyro-sensor/htway/ for details.
 * </p>
 * 
 * @author Brian Bagnall
 * @author Barry Becker
 */
class WheelController {
    
    private EncoderMotor left_motor; 
    private EncoderMotor right_motor;

    /**
     * Power differential used for steering based on difference of target steering and actual motor difference.
     */
    private static final double KSTEER = 0.25;

    /**
     * This constant is in degrees/second for maximum speed.  Note that position 
     * and speed are measured as the sum of the two motors, in other words, 600 
     * would actually be 300 degrees/second for each motor.
     */
    private static final double CONTROL_SPEED  = 600.0;
    
    /**
     * motorControlDrive is the target speed for the sum of the two motors
     * in degrees per second.
     */
    private double motorControlDrive = 0.0;
    
    /**
     * motorControlSteer is the target change in difference for two motors
     * in degrees per second.
     */
    private double motorControlSteer = 0.0;

    /**
     * This global contains the target motor differential, essentially, which 
     * way the robot should be pointing.  This value is updated every time through 
     * the balance loop based on motorControlSteer.
     */
    private double motorDiffTarget = 0.0;

    // Motor globals
    private double motorPos = 0;
    private long mrcSum = 0, mrcSumPrev;
    private long motorDiff;
    private long mrcDeltaP3 = 0;
    private long mrcDeltaP2 = 0;
    private long mrcDeltaP1 = 0;
    
    private double motorSpeed;
    
    /** 
     * Control the amount of power to apply to each wheel.
     * Updated by the steerControl() method.
     */
    private int powerLeft, powerRight; 
   

    /**
     * Constructor
     * @param left The left motor. An unregulated motor.
     * @param right The right motor. An unregulated motor.
     */
    WheelController(EncoderMotor left, EncoderMotor right) {
        this.left_motor = left;
        this.right_motor = right;
    }
    
    /**
     * Allows the robot to move forward/backward and make in-spot rotations as
     * well as arcs by varying the power to each wheel. This method does not actually 
     * apply direct power to the wheels. Control is filtered through to each wheel, allowing the robot
     * drive forward/backward and make turns. Higher values are faster. Negative values cause the wheel
     * to rotate backwards. Values between -200 and 200 are good. If values are too high it can make the
     * robot balance unstable.
     * 
     * @param left_wheel The relative control power to the left wheel. -200 to 200 are good numbers.
     * @param right_wheel The relative control power to the right wheel. -200 to 200 are good numbers.
     */
    public void wheelDriver(int left_wheel, int right_wheel) {
        // Set control Drive and Steer.  Both these values are in motor degree/second
        motorControlDrive = (left_wheel + right_wheel) * CONTROL_SPEED / 200.0;
        motorControlSteer = (left_wheel - right_wheel) * CONTROL_SPEED / 200.0;
    }  
    
    /**
     * @return The motor position used for balancing.
     *    Note that this variable has two sources of input:
     *    Change in motor position based on the sum of
     *    MotorRotationCount of the two motors, and
     *    forced movement based on user driving the robot.
     *    Units: degrees (sum of the two motors)
     */
    public double getMotorPos() {
    	return motorPos;
    }
    
    public double getMotorControlDrive() {
    	return motorControlDrive;
    }
    
    /**
     * @return the speed of the wheels of the robot based on the motor encoders.
     *         Units: degrees/second (sum of the two motors)
     */
    public double getMotorSpeed() {
    	return motorSpeed;
    }
    
    /**
     * Reset the motors to make sure we start at a zero position
     */
    public void resetTachoCount() {
        left_motor.resetTachoCount();
        right_motor.resetTachoCount();
    }
    
    public void floatMotors() {
    	left_motor.flt();
        right_motor.flt();
    }

    /**
     * Keeps track of wheel position for both motors.
     * @tInterval time delta since last update.
     */
    public void updateMotorData(double tInterval) {
        long mrcLeft, mrcRight, mrcDelta;

        // Keep track of motor position and speed
        mrcLeft = left_motor.getTachoCount();
        mrcRight = right_motor.getTachoCount();

        // Maintain previous mrcSum so that delta can be calculated and get
        // new mrcSum and Diff values
        mrcSumPrev = mrcSum;
        mrcSum = mrcLeft + mrcRight;
        motorDiff = mrcLeft - mrcRight;

        // mrcDetla is the change int sum of the motor encoders, update
        // motorPos based on this delta
        mrcDelta = mrcSum - mrcSumPrev;
        motorPos += mrcDelta;

        // motorSpeed is based on the average of the last four delta's.
        motorSpeed = (mrcDelta + mrcDeltaP1 + mrcDeltaP2 + mrcDeltaP3) / (4 * tInterval);

        // Shift the latest mrcDelta into the previous three saved delta values
        mrcDeltaP3 = mrcDeltaP2;
        mrcDeltaP2 = mrcDeltaP1;
        mrcDeltaP1 = mrcDelta;
        
        // Apply the drive control value to the motor position to get robot to move.
        motorPos -= motorControlDrive * tInterval;
    }

    /**
     * This function determines the left and right motor power that should
     * be used based on the balance power and the steering control.
     */
    public void steerControl(int power, double tInterval) {
        int powerSteer;

        // Update the target motor difference based on the user steering
        // control value.
        motorDiffTarget += motorControlSteer * tInterval;

        // Determine the proportionate power differential to be used based
        // on the difference between the target motor difference and the
        // actual motor difference.
        powerSteer = (int)(KSTEER * (motorDiffTarget - motorDiff));

        // Apply the power steering value with the main power value to
        // get the left and right power values.
        powerLeft = power + powerSteer;
        powerRight = power - powerSteer;

        // Limit the power to motor power range -100 to 100
        if (powerLeft > 100)   powerLeft = 100;
        if (powerLeft < -100)  powerLeft = -100;

        // Limit the power to motor power range -100 to 100
        if (powerRight > 100)  powerRight = 100;
        if (powerRight < -100) powerRight = -100;
        
        setLeftPower(powerLeft);
        setRightPower(powerRight);
    }
    
    private void setLeftPower(int power) {
    	left_motor.setPower(Math.abs(power));
    	 if(power > 0) left_motor.forward(); 
         else left_motor.backward();
    }
    
    private void setRightPower(int power) {
    	right_motor.setPower(Math.abs(power));
    	if(power > 0) right_motor.forward(); 
        else right_motor.backward();
    }
}