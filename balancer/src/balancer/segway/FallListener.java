package balancer.segway;

/**
 * Implemented by a class that needs to b notified when an entity falls.
 * 
 * @author Barry Becker
 */
public interface FallListener {

	/**
	 * Called when the entity fell.
	 * @param elapsedTime time it took to fall in milliseconds.
	 */
    void hasFallen(double elapsedTime);
}
