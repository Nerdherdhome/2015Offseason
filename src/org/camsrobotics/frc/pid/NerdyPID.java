package org.camsrobotics.frc.pid;

/**
 * PID Calculation class
 * 
 * @author Wesley
 *
 */
public class NerdyPID {
	private final double m_kP, m_kI, m_kD;
	private double m_desired;
	private double m_error = 0;
	private double m_lastError;
	private NerdyIntegrator m_integrator;
	
	/**
	 * Default constructor
	 * 
	 * @param kP P constant value
	 * @param kI I constant value
	 * @param kD D constant value
	 */
	public NerdyPID(double kP, double kI, double kD)	{
		m_kP = kP;
		m_kI = kI;
		m_kD = kD;
		m_integrator = new NerdyIntegrator();
	}
	
	/**
	 * Sets the desired value
	 * 
	 * @param desired
	 * @return this
	 */
	public NerdyPID setDesired(double desired)	{
		m_desired = desired;
		return this;
	}
	
	/**
	 * Resets the calculations
	 */
	public void reset()	{
		m_integrator.reset();
		m_error = 0;
		m_lastError = 0;
	}
	
	/**
	 * Calculates the power value
	 * 
	 * @param heading The sensor input
	 * @return The power value
	 */
	public double calculate(double heading)	{
		m_lastError = m_error;
		m_error = heading - m_desired;
		
		// P calculation
		double p = m_error * m_kP;
		
		// I calculation
		m_integrator.integrate(m_error);
		double i = m_integrator.get() * m_kI;
		
		// D calculation
		double d = m_kD * (m_error - m_lastError);
		
		// put it together
		return p+i-d;
	}
}
