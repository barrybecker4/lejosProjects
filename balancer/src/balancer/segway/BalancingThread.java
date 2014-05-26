package balancer.segway;

import lejos.robotics.EncoderMotor;

/**
 * <p>This class balances a two-wheeled robot. 
 * Refer to http://www.hitechnic.com/blog/gyro-sensor/htway/ for details.
 * 
 * <p>NOTE: In order to make the robot move and navigate, use the SegwayPilot class.</p> 
 * <p>
 * <i>This code is based on the <a href="http://www.hitechnic.com/blog/gyro-sensor/htway/">HTWay</a> by HiTechnic.</i>
 * </p>
 * 
 * @author Brian Bagnall
 * @author Barry Becker
 */
class BalancingThread extends Thread {

    private GyroSensor gyro;
    private WheelController wheelController;
    
    /**
     * This constant aids in drive control. When the robot starts moving because of user control,
     * this constant helps get the robot leaning in the right direction.  Similarly, it helps 
     * bring robot to a stop when stopping.
     */
    private static final double KDRIVE = -0.02;
 
    /** 
     * Loop wait time.  WAIT_TIME is the time in ms passed to the Wait command.
     * NOTE: Balance control loop only takes 1.128 MS in leJOS NXJ. 
     */
    private static final int WAIT_TIME = 7; // originally 8
    
    // These are the main four balance constants, only the gyro
    // constants are relative to the wheel size.  KPOS and KSPEED
    // are self-relative to the wheel size.
    private static final double KGYROANGLE = 7.5;
    private static final double KGYROSPEED = 1.15;
    private static final double KPOS = 0.07;
    private static final double KSPEED = 0.1;

    /** 
     * If robot power is saturated (over +/- 100) for over this time limit then 
     * robot must have fallen.  In milliseconds.
     */
    private static final double TIME_FALL_LIMIT = 500;
    
    /** First time through, set an initial tInterval time. */
    private static final double INITIAL_INTERVAL_TIME = 0.0055;

    /** Time that robot first starts to balance.  Used to calculate tInterval. */
    private long tCalcStart;

    /** tInterval is the time, in seconds, for each iteration of the balance loop. */
    private double tInterval;
    
    /** total time that the robot has been balancing. */
    private double totalTime = 0;

    /**
     * ratioWheel stores the relative wheel size compared to a standard NXT 1.0 wheel.
     * RCX 2.0 wheel has ratio of 0.7 while large RCX wheel is 1.4.
     */
    private double ratioWheel;
    
    /** thing to notify upon falling */
    private FallListener fallListener;
    
    /** Set to true when the thread is to stop. Use instead of calling stop(). */
    private boolean terminated = false;

    /**
     * Creates an instance of the Segway, prompts the user to lay Segway flat for gyro calibration,
     * then begins self-balancing thread. Wheel diameter is used in balancing equations.
     *  
     *  <li>NXT 1.0 wheels = 5.6 cm
     *  <li>NXT 2.0 wheels = 4.32 cm
     *  <li>RCX "motorcycle" wheels = 8.16 cm
     * 
     * @param left The left motor. An unregulated motor.
     * @param right The right motor. An unregulated motor.
     * @param gyro gyroscopic sensor
     * @param wheelDiameter diameter of wheel, preferably use cm (printed on side of LEGO tires in mm)
     * @param fallListener something to notify when the robot falls.
     */
    BalancingThread(EncoderMotor left, EncoderMotor right, 
    		GyroSensor gyro, double wheelDiameter, FallListener fallListener) {
    	
    	wheelController = new WheelController(left, right);
    
        // Optional code to accept BasicMotor: this.right_motor = (NXTMotor)right;
        this.gyro = gyro;
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
    	wheelController.wheelDriver(left_wheel, right_wheel);
    }  
    
    /**
     * Tell this thread to stop running. Do not call the deprecated stop method on Thread.
     */
    public void terminate() {
        terminated = true;
    }
    
    /** 
     * This is the main balance thread for the robot. Robot is assumed to start balanced. 
     *
     * After an initial gyro offset is established, the robot backs up
     * against the wall until it falls forward, when it detects the
     * forward fall, it start the balance loop.
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
                
        tMotorPosOK = System.currentTimeMillis();

        wheelController.resetTachoCount();

        // NOTE: This balance control loop only takes 1.128 MS to execute each loop in leJOS NXJ.
        while (!terminated) {
            calcInterval(cLoop++);

            gyro.updateGyroData(tInterval);
            wheelController.updateMotorData(tInterval);
            
            // This is the main balancing equation
            power = (int)((KGYROSPEED * gyro.getAngularVelocity() +          // Deg/Sec from Gyro sensor
                    KGYROANGLE * gyro.getAngle()) / ratioWheel +   // Deg from integral of gyro
                    KPOS       * wheelController.getMotorPos() +   // From MotorRotationCount of both motors
                    KDRIVE     * wheelController.getMotorControlDrive() +         // To improve start/stop performance
                    KSPEED     * wheelController.getMotorSpeed());                // Motor speed in Deg/Sec

            if (Math.abs(power) < 100) {
                tMotorPosOK = System.currentTimeMillis();
            }
            // Movement control. Not used for balancing.
            wheelController.steerControl(power, tInterval); 

            // Check if robot has fallen by detecting that motorPos is being limited
            // for an extended amount of time.
            if ((System.currentTimeMillis() - tMotorPosOK) > TIME_FALL_LIMIT) {
            	terminated = true;
            }
            
            ThreadUtil.sleep(WAIT_TIME);
        }
        
        wheelController.floatMotors();
        fallListener.hasFallen(totalTime);
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
            tInterval = INITIAL_INTERVAL_TIME;
            tCalcStart = System.currentTimeMillis();
        } else {
            // Take average of number of times through the loop and
            // use for interval time.
            tInterval = (System.currentTimeMillis() - tCalcStart) / (cLoop * 1000.0);
        }
        totalTime += tInterval;
    }
}