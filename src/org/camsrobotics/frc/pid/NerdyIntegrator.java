package org.camsrobotics.frc.pid;

/**
 * Trapezoidal integrator
 * 
 * @author Wesley
 *
 */
public class NerdyIntegrator {
	private double m_integration;
	private double m_lastTime;
	private double m_lastVal;
	
	/**
	 * Default constructor
	 */
	public NerdyIntegrator(){
		m_lastTime = System.currentTimeMillis();
		m_integration = 0;
	}
	
	/**
	 * Constructs an integrator with a custom starting point
	 * 
	 * @param start The starting point
	 */
	public NerdyIntegrator(double start)	{
		m_lastTime = System.currentTimeMillis();
		m_integration = start;
	}
	
	/**
	 * Adds a value to the integrator
	 * 
	 * @param val The value
	 */
	public void integrate(double val)	{
		double time = System.currentTimeMillis();
		
		m_integration += ((val+m_lastVal)/2)*(time-m_lastTime);
		
		m_lastTime = time;
		m_lastVal = val;
	}
	
	/**
	 * Gets the total integrated value
	 * 
	 * @return The integrated value
	 */
	public double get()	{
		return m_integration;
	}
	
	/**
	 * Resets the integration
	 */
	public void reset()	{
		m_integration = 0;
	}
}
