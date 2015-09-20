package org.camsrobotics.frc.subsystems;

import org.camsrobotics.frc.util.NerdyStates;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Talon;

/**
 * Intake Controller
 * 
 * @author Wesley
 *
 */
public class NerdyIntake {
	/**
	 * Intake States
	 * 
	 * @author Wesley
	 *
	 */
	private static class State	{
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
	
	private Talon m_leftIntake, m_rightIntake;
	private DoubleSolenoid m_solenoid;
	private double power = .75;
	private int m_spinMode = 0;
	private boolean m_solMode = true;
	private State m_state;
	
	/**
	 * Default constructor
	 * 
	 * @param leftIntake
	 * @param rightIntake
	 * @param sol
	 */
	public NerdyIntake(Talon leftIntake, Talon rightIntake, DoubleSolenoid sol)	{
		m_leftIntake	= leftIntake;
		m_rightIntake	= rightIntake;
		m_solenoid		= sol;
		m_state			= State.kOn;
		NerdyStates.put("Intake", m_state);
	}
	
	/**
	 * Disables the intakes
	 */
	public void disable()	{
		m_state = State.kOff;
		NerdyStates.put("Intake", m_state);
	}
	
	/**
	 * Enables the intakes
	 */
	public void enable()	{
		m_state = State.kOn;
		NerdyStates.put("Intake", m_state);
	}
	
	/**
	 * Intake a tote
	 */
	public void intake()	{
		m_spinMode = 1;
	}
	
	/**
	 * Outake a tote
	 */
	public void outake()	{
		m_spinMode = -1;
	}
	
	/**
	 * Do nothing
	 */
	public void idle()	{
		m_spinMode = 0;
	}
	
	/**
	 * Open the intakes
	 */
	public void open()	{
		m_solMode = true;
	}
	
	/**
	 * Close the intakes
	 */
	public void close()	{
		m_solMode = false;
	}
	
	/**
	 * Runs the intakes. Should be run every iteration
	 */
	public void run()	{
		if(m_state == State.kOn)	{
			if(m_spinMode == 1)	{
				m_leftIntake.set(power);
				m_rightIntake.set(-power);
			}	else if(m_spinMode == -1)	{
				m_leftIntake.set(-power);
				m_rightIntake.set(power);
			}	else if(m_spinMode == 0)	{
				m_leftIntake.set(0);
				m_rightIntake.set(0);
			}
			
			if(m_solMode)	{
				m_solenoid.set(DoubleSolenoid.Value.kForward);
			}	else	{
				m_solenoid.set(DoubleSolenoid.Value.kReverse);
			}
		}
	}
}
