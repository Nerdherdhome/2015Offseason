package org.camsrobotics.frc.util;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

public class NerdyDrivebase {
	/**
	 * Drivebase States
	 * 
	 * @author Wesley
	 *
	 */
	public static class State	{
        public final int state;
        private static final int kOff_val = -1;
        private static final int kOn_val = 0;
        
        public static final State kOff = new State(kOff_val);
        public static final State kOn = new State(kOn_val);
        
        private State(int state) {
            this.state = state;
        }
        
        public String toString()	{
        	switch(state)	{
        	case 0:
        		return "Off";
			case 1:
        		return "On";
        	}
        	// Should never happen
			return null;
        }
	}
	private SpeedController m_ftLeft, m_ftRight, m_bkLeft, m_bkRight;
	private Encoder m_ftLeftEncoder, m_ftRightEncoder, m_bkLeftEncoder, m_bkRightEncoder;
	private State m_state;
	
	/**
	 * Default constructor
	 * 
	 * @param ftLeft
	 * @param ftRight
	 * @param bkLeft
	 * @param bkRight
	 * @param ftLeftEnc
	 * @param ftRightEnc
	 * @param bkLeftEnc
	 * @param bkRightEnc
	 */
	public NerdyDrivebase(SpeedController ftLeft, SpeedController ftRight, SpeedController bkLeft, SpeedController bkRight,
			Encoder ftLeftEnc, Encoder ftRightEnc, Encoder bkLeftEnc, Encoder bkRightEnc)	{
		m_ftLeft = ftLeft;
		m_ftRight = ftRight;
		m_bkLeft = bkLeft;
		m_bkRight = bkRight;
		m_ftLeftEncoder = ftLeftEnc;
		m_ftRightEncoder = ftRightEnc;
		m_bkLeftEncoder = bkLeftEnc;
		m_bkRightEncoder = bkRightEnc;
		m_state = State.kOn;
	}
	
	/**
	 * Disables the drivebase
	 */
	public void disable()	{
		m_state = State.kOff;
	}
	
	/**
	 * Enables the drivebase
	 */
	public void enable()	{
		m_state = State.kOn;
	}
	
	/**
	 * Resets the encoder values
	 */
	public void reset()	{
		m_ftLeftEncoder.reset();
		m_ftRightEncoder.reset();
		m_bkLeftEncoder.reset();
		m_bkRightEncoder.reset();
	}
	
	public int getFtLeft()	{
		return m_ftLeftEncoder.get();
	}
	
	public int getFtRight()	{
		return m_ftRightEncoder.get();
	}
	
	public int getBkLeft()	{
		return m_bkLeftEncoder.get();
	}
	
	public int getBkRight()	{
		return m_bkRightEncoder.get();
	}
	
	public void drive(double ftLeft, double ftRight, double bkLeft, double bkRight)	{
		if(m_state == State.kOn)	{
			m_ftLeft.set(NerdyMath.threshold(ftLeft, -1, 1));
			m_ftRight.set(NerdyMath.threshold(ftRight, -1, 1));
			m_bkLeft.set(NerdyMath.threshold(bkLeft, -1, 1));
			m_bkRight.set(NerdyMath.threshold(bkRight, -1, 1));
		}	else	{
			m_ftLeft.set(0);
			m_ftRight.set(0);
			m_bkLeft.set(0);
			m_bkRight.set(0);
		}
	}
}
