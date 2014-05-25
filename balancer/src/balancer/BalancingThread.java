package balancer;

import lejos.hardware.sensor.HiTechnicGyro;
import lejos.robotics.EncoderMotor;

/**
 * <p>This class balances a two-wheeled robot. 
 * Refer to http://www.hitechnic.com/blog/gyro-sensor/htway/ for details.
 * 
 * <p>NOTE: In order to make the robot move and navigate, use the SegowayPilot class.</p> 
 * 
 * <p><i>This code is based on the <a href="http://www.hitechnic.com/blog/gyro-sensor/htway/">HTWay</a> by HiTechnic.</i></p>
 * 
 * @author Brian Bagnall
 * @author Barry Becker
 */
public class BalancingThread extends Thread {

    // Gyro sensor
    private HiTechnicGyro gyro;
 
    // baseline value used for calibration. 0 until it is determined.
    private float gyroBaseline = 0;
    
    private EncoderMotor left_motor;   // used to be EncoderMotor
    private EncoderMotor right_motor;
    
    //=====================================================================
    // Balancing constants
    //
    // These are the constants used to maintain balance.
    //=====================================================================
    
    /** 
     * Loop wait time.  WAIT_TIME is the time in ms passed to the Wait command.
     * NOTE: Balance control loop only takes 1.128 MS in leJOS NXJ. 
     */
    private static final int WAIT_TIME = 6; // originally 8
    
    // These are the main four balance constants, only the gyro
    // constants are relative to the wheel size.  KPOS and KSPEED
    // are self-relative to the wheel size.
    private static final double KGYROANGLE = 7.5;
    private static final double KGYROSPEED = 1.15;
    private static final double KPOS = 0.07;
    private static final double KSPEED = 0.1;

    /**
     * This constant aids in drive control. When the robot starts moving because of user control,
     * this constant helps get the robot leaning in the right direction.  Similarly, it helps 
     * bring robot to a stop when stopping.
     */
    private static final double KDRIVE = -0.02;

    /**
     * Power differential used for steering based on difference of target steering and actual motor difference.
     */
    private static final double KSTEER = 0.25;

    /**
     * Gyro offset control
     * The gyro sensor will drift with time.  This constant is used in a simple long term averaging
     * to adjust for this drift. Every time through the loop, the current gyro sensor value is
     * averaged into the gyro offset weighted according to this constant.
     */
    private static final double EMAOFFSET = 0.0004;  // originally 0.0005

    /** 
     * If robot power is saturated (over +/- 100) for over this time limit then 
     * robot must have fallen.  In milliseconds.
     */
    private static final double TIME_FALL_LIMIT = 500; // originally 1000


    /**
     * This constant is in degrees/second for maximum speed.  Note that position 
     * and speed are measured as the sum of the two motors, in other words, 600 
     * would actually be 300 degrees/second for each motor.
     */
    private static final double CONTROL_SPEED  = 600.0;

    //=====================================================================
    // Global variables
    //=====================================================================
    
    // These two xxControlDrive variables are used to control the movement of the robot. Both
    // are in degrees/second:    
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

    /**
     * Time that robot first starts to balance.  Used to calculate tInterval.
     */
    private long tCalcStart;

    /**
     * tInterval is the time, in seconds, for each iteration of the balance loop.
     */
    private double tInterval;

    /**
     * ratioWheel stores the relative wheel size compared to a standard NXT 1.0 wheel.
     * RCX 2.0 wheel has ratio of 0.7 while large RCX wheel is 1.4.
     */
    private double ratioWheel;

    // Gyro globals
    private double gOffset;
    private double gAngleGlobal = 0;

    // Motor globals
    private double motorPos = 0;
    private long mrcSum = 0, mrcSumPrev;
    private long motorDiff;
    private long mrcDeltaP3 = 0;
    private long mrcDeltaP2 = 0;
    private long mrcDeltaP1 = 0;

    private double gyroSpeed, gyroAngle; // originally local variables
    private double motorSpeed; // originally local variable
    
    /** 
     * Global variables used to control the amount of power to apply to each wheel.
     * Updated by the steerControl() method.
     */
    private int powerLeft, powerRight; 
    
    private FallListener fallListener;
    private boolean terminated = false;

    /**
     * Creates an instance of the Segoway, prompts the user to lay Segoway flat for gyro calibration,
     * then begins self-balancing thread. Wheel diameter is used in balancing equations.
     *  
     *  <li>NXT 1.0 wheels = 5.6 cm
     *  <li>NXT 2.0 wheels = 4.32 cm
     *  <li>RCX "motorcycle" wheels = 8.16 cm
     * 
     * @param left The left motor. An unregulated motor.
     * @param right The right motor. An unregulated motor.
     * @param gyro A HiTechnic gyro sensor
     * @param gyroBase calibration offset
     * @param wheelDiameter diameter of wheel, preferably use cm (printed on side of LEGO tires in mm)
     * @param fallListener something to notify when the robot falls.
     */
    public BalancingThread(EncoderMotor left, EncoderMotor right, 
    		HiTechnicGyro gyro, float gyroBase, double wheelDiameter, FallListener fallListener) {
        this.left_motor = left;
        this.right_motor = right;
        // Optional code to accept BasicMotor: this.right_motor = (NXTMotor)right;
        this.gyro = gyro;
        this.gyroBaseline = gyroBase;
        // Original algorithm was tuned for 5.6 cm NXT 1.0 wheels.
        this.ratioWheel = wheelDiameter / 5.6; 
        
        this.fallListener = fallListener;
        this.setDaemon(true);
    }
    
    /**
     * This method allows the robot to move forward/backward and make in-spot rotations as
     * well as arcs by varying the power to each wheel. This method does not actually 
     * apply direct power to the wheels. Control is filtered through to each wheel, allowing the robot to 
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
    
    public void terminate() {
        terminated = true;
    }
    
    /** 
     * This is the main balance thread for the robot.
     *
     * Robot is assumed to start leaning on a wall.  The first thing it
     * does is take multiple samples of the gyro sensor to establish and
     * initial gyro offset.
     *
     * After an initial gyro offset is established, the robot backs up
     * against the wall until it falls forward, when it detects the
     * forward fall, it start the balance loop.
     *
     * The main state variables are:
     * gyroAngle  This is the angle of the robot, it is the results of
     *            integrating on the Gyro value.
     *            Units: degrees
     * gyroSpeed  The value from the Gyro Sensor after offset subtracted
     *            Units: degrees/second
     * motorPos   This is the motor position used for balancing.
     *            Note that this variable has two sources of input:
     *             Change in motor position based on the sum of
     *             MotorRotationCount of the two motors,
     *            and,
     *             forced movement based on user driving the robot.
     *            Units: degrees (sum of the two motors)
     * motorSpeed This is the speed of the wheels of the robot based on the
     *            motor encoders.
     *            Units: degrees/second (sum of the two motors)
     *
     * From these state variables, the power to the motors is determined
     * by this linear equation:
     *     power = KGYROSPEED * gyro +
     *             KGYROANGLE * gyroAngle +
     *             KPOS       * motorPos +
     *             KSPEED     * motorSpeed;
     */
    public void run() {

        int power;
        long tMotorPosOK;
        long cLoop = 0;
                
        //System.out.println("Balancing");
        //System.out.println();
        
        tMotorPosOK = System.currentTimeMillis();

        // Reset the motors to make sure we start at a zero position
        left_motor.resetTachoCount();
        right_motor.resetTachoCount();

        // NOTE: This balance control loop only takes 1.128 MS to execute each loop in leJOS NXJ.
        while (!terminated) {
            calcInterval(cLoop++);

            updateGyroData();
            updateMotorData();

            // Apply the drive control value to the motor position to get robot to move.
            motorPos -= motorControlDrive * tInterval;

            // This is the main balancing equation
            power = (int)((KGYROSPEED * gyroSpeed +               // Deg/Sec from Gyro sensor
                    KGYROANGLE * gyroAngle) / ratioWheel + // Deg from integral of gyro
                    KPOS       * motorPos +                 // From MotorRotaionCount of both motors
                    KDRIVE     * motorControlDrive +        // To improve start/stop performance
                    KSPEED     * motorSpeed);                // Motor speed in Deg/Sec

            if (Math.abs(power) < 100)
                tMotorPosOK = System.currentTimeMillis();

            steerControl(power); // Movement control. Not used for balancing.

            // Apply the power values to the motors
            // NOTE: It would be easier/faster to use MotorPort.controlMotorById(), but it needs to be public.
            left_motor.setPower(Math.abs(powerLeft));
            right_motor.setPower(Math.abs(powerRight));

            if(powerLeft > 0) left_motor.forward(); 
            else left_motor.backward();

            if(powerRight > 0) right_motor.forward(); 
            else right_motor.backward();

            // Check if robot has fallen by detecting that motorPos is being limited
            // for an extended amount of time.
            if ((System.currentTimeMillis() - tMotorPosOK) > TIME_FALL_LIMIT) break;
            
            try {Thread.sleep(WAIT_TIME);} catch (InterruptedException e) {}
        } // end of while() loop
        
        left_motor.flt();
        right_motor.flt();

        fallListener.hasFallen(tInterval);
    } 

    /**
     * Get the data from the gyro. 
     * Fills the pass by reference gyroSpeed and gyroAngle based on updated information from the Gyro Sensor.
     * Maintains an automatically adjusted gyro offset as well as the integrated gyro angle.
     */
    private void updateGyroData() {
        
        float gyroRaw = getAngularVelocity();
        gOffset = EMAOFFSET * gyroRaw + (1-EMAOFFSET) * gOffset;
        gyroSpeed = gyroRaw - gOffset; // Angular velocity (degrees/sec)

        gAngleGlobal += gyroSpeed * tInterval;
        gyroAngle = gAngleGlobal; // Absolute angle (degrees)
    }
    
    /**
     * @return the calibrated angular velocity from the gyro sensor.
     */
    private float getAngularVelocity() {
        float[] rate = new float[1];
        gyro.fetchSample(rate, 0);
        float rotSpeed = rate[0] - gyroBaseline;
        return rotSpeed; 
    }

    /**
     * Keeps track of wheel position for both motors.
     */
    private void updateMotorData() {
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
        motorSpeed = (mrcDelta + mrcDeltaP1+mrcDeltaP2+mrcDeltaP3) / (4 * tInterval);

        // Shift the latest mrcDelta into the previous three saved delta values
        mrcDeltaP3 = mrcDeltaP2;
        mrcDeltaP2 = mrcDeltaP1;
        mrcDeltaP1 = mrcDelta;
    }

    /**
     * This function determines the left and right motor power that should
     * be used based on the balance power and the steering control.
     */
    private void steerControl(int power) {
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
    }

    /**
     * Calculate the interval time from one iteration of the loop to the next.
     * Note that first time through, cLoop is 0, and has not gone through
     * the body of the loop yet.  Use it to save the start time.
     * After the first iteration, take the average time and convert it to
     * seconds for use as interval time.
     */
    private void calcInterval(long cLoop) {
        if (cLoop == 0) {
            // First time through, set an initial tInterval time and
            // record start time
            tInterval = 0.0055;
            tCalcStart = System.currentTimeMillis();
        } else {
            // Take average of number of times through the loop and
            // use for interval time.
            tInterval = (System.currentTimeMillis() - tCalcStart) / (cLoop * 1000.0);
        }
    }
}