package balancer.segway;

/** 
 * Common static methods when dealing with threads.
 * @author Barry Becker
 */
public class ThreadUtil {
    
    public static void sleep(int millis) {
    	try { 
    		Thread.sleep(millis);
        } 
    	catch (InterruptedException e) {
    		e.printStackTrace();
    	}
    }
    
    /** do not instantiate private class */
    private ThreadUtil() {}
}
