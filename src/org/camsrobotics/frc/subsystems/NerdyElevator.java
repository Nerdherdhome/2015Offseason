package org.camsrobotics.frc.subsystems;

import org.camsrobotics.frc.pid.NerdyPID;
import org.camsrobotics.frc.util.NerdyJoystick;
import org.camsrobotics.frc.util.NerdyMath;
import org.camsrobotics.frc.util.NerdyResources;
import org.camsrobotics.frc.util.NerdyStates;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Elevator controller
 * 
 * @author Wesley
 *
 */
public class NerdyElevator {
	/**
	 * Elevator States
	 * 
	 * @author Wesley
	 *
	 */
	private static class State	{
        public final int state;
        private static final int kOff_val = -1;
        private static final int kManual_val = 0;
        private static final int kAutoStack_val = 2;
        private static final int kAutoDrive_val = 3;
        private static final int kCommand_val = 4;
        
        public static final State kOff = new State(kOff_val);
        public static final State kManual = new State(kManual_val);
        public static final State kAutoStack = new State(kAutoStack_val);
        public static final State kAutoDrive = new State(kAutoDrive_val);
        public static final State kCommand = new State(kCommand_val);

        private State(int state) {
            this.state = state;
        }
        
        public String toString()	{
        	switch(state)	{
        	case -1:
        		return "Off";
			case 0:
        		return "Manual Mode";
			case 2:
				return "Automatic Mode";
			case 3:
				return "Automatic Mode";
			case 4:
				return "Command Mode";
        	}
        	// Should never happen
			return null;
        }
	}
	
	// Hardware
	private NerdyJoystick m_joy;
	private CANTalon m_lift;
	private DoubleSolenoid m_left, m_right, m_claw;
	private Encoder m_encoder;
	
	private State m_state;
	private NerdyPID m_pid;
	
	private boolean m_open = true, m_clawOpen = false;
	private double m_encode = 8;
	
	// For command mode
	private double m_desired;
	
	private int m_autoStackState = -1;
	
	/**
	 * Default constructor
	 * 
	 * @param joy
	 * @param lift
	 * @param leftSol
	 * @param rightSol
	 * @param encoder
	 * @param lowerLimit
	 * @param upperLimit
	 */
	public NerdyElevator(NerdyJoystick joy,
			CANTalon lift,
			DoubleSolenoid leftSol,
			DoubleSolenoid rightSol,
			DoubleSolenoid claw,
			Encoder encoder)	{
		m_joy			= joy;
		m_lift			= lift;
		m_left			= leftSol;
		m_right			= rightSol;
		m_claw			= claw;
		m_encoder		= encoder;
		
		m_pid			= new NerdyPID(NerdyResources.Constants.articLiftP,
						   NerdyResources.Constants.articLiftI,
						   NerdyResources.Constants.articLiftD);
		
		m_state = State.kManual;
		NerdyStates.put("Elevator", m_state);
	}
	
	/**
	 * Disables the subsystem
	 */
	public void disable()	{
		m_state = State.kOff;
		NerdyStates.put("Elevator", m_state);
	}
	
	/**
	 * Enables the subsystem. Default state: Manual mode
	 */
	public void enable()	{
		m_state = State.kManual;
		NerdyStates.put("Elevator", m_state);
	}
	
	/**
	 * Switches to manual mode
	 */
	public void manual()	{
		m_state = State.kManual;
		NerdyStates.put("Elevator", m_state);
	}
	
	/**
	 * Autostacks to the stacking height
	 */
	public void stackToStack()	{
		m_state = State.kAutoStack;
		m_autoStackState = 0;
		NerdyStates.put("Articulation", m_state);
	}
	
	/**
	 * Autostacks a tote higher
	 */
	public void stackToStack2()	{
		m_state = State.kAutoDrive;
		m_autoStackState = 0;
		NerdyStates.put("Articulation", m_state);
	}
	
	/**
	 * Switches to command mode (if applicable) and sets the desired height
	 * 
	 * @param desired The number of inches off the floor
	 * 
	 */
	public void command(double desired)	{
		if(m_state != State.kCommand)	{
			m_state = State.kCommand;
			NerdyStates.put("Articulation", m_state);
		}
	}
	
	/**
	 * Opens the arms
	 */
	public void open()	{
		m_open = true;
	}
	
	/**
	 * Closes the arms
	 */
	public void close()	{
		m_open = false;
	}
	
	/**
	 * Opens the claw
	 */
	public void openClaw()	{
		m_clawOpen = true;
	}
	
	/**
	 * Closes the claw
	 */
	public void closeClaw()	{
		m_clawOpen = false;
	}
	
	/**
	 * Calibrates the elevator
	 */
	public void calibrate()	{
		m_encoder.reset();
	}
	
	/**
	 * Runs the motors with the calculated values. Run this function every iteration.
	 */
	public void run()	{
		double pow = 0;
		
		// Sets m_encode to the height (inches) of the arms off the ground
		m_encode = m_encoder.getRaw()/256 + 8;
		
		if(m_state != State.kOff)	{
			if(m_state == State.kManual)	{
				SmartDashboard.putBoolean("AutoStacking?", false);
				pow = m_joy.getTrueY();
			}	else if(m_state == State.kAutoStack)	{
				SmartDashboard.putBoolean("AutoStacking?", true);
				double[] heights = {NerdyResources.Constants.stackHeight1,
						NerdyResources.Constants.stackHeight2, 
						NerdyResources.Constants.stackHeight3};
				switch(m_autoStackState)	{
				case 0:
					pow = m_pid.setDesired(heights[0]).calculate(m_encode);
					if(Math.abs(m_encode - heights[0])<1)	{
						m_autoStackState = 1;
						m_pid.reset();
					}
					break;
				case 1:
					m_left.set(DoubleSolenoid.Value.kReverse);
					m_right.set(DoubleSolenoid.Value.kReverse);
						m_autoStackState = 2;
					break;
				case 2:
					pow = m_pid.setDesired(heights[1]).calculate(m_encode);
					if(Math.abs(m_encode - heights[1])<1)	{
						m_autoStackState = 3;
						m_pid.reset();
						pow = 0;
					}
					break;
				case 3:
					pow = 0;
					m_left.set(DoubleSolenoid.Value.kForward);
					m_right.set(DoubleSolenoid.Value.kForward);
					Timer.delay(NerdyResources.Constants.pause);
					m_autoStackState = 4;
					m_open = true;
					break;
				case 4:
					pow = m_pid.setDesired(heights[2]).calculate(m_encode);
					if(Math.abs(m_encode - heights[2])<1)	{
						m_autoStackState = -1;
						manual();
						m_pid.reset();
					}
					break;
				default:
					manual();
					break;
				}
			}	else if(m_state == State.kAutoDrive)	{
				SmartDashboard.putBoolean("AutoStacking?", true);
				double[] heights = {NerdyResources.Constants.stackHeight21,
						NerdyResources.Constants.stackHeight22, 
						NerdyResources.Constants.stackHeight23};
				switch(m_autoStackState)	{
				case 0:
					pow = m_pid.setDesired(heights[0]).calculate(m_encode);
					if(Math.abs(m_encode - heights[0])<.1)	{
						m_autoStackState = 1;
						m_pid.reset();
					}
					break;
				case 1:
					m_left.set(DoubleSolenoid.Value.kReverse);
					m_right.set(DoubleSolenoid.Value.kReverse);
						m_autoStackState = 2;
					break;
				case 2:
					pow = m_pid.setDesired(heights[1]).calculate(m_encode);
					if(Math.abs(m_encode - heights[1])<.1)	{
						m_autoStackState = 3;
						m_pid.reset();
						pow = 0;
					}
					break;
				case 3:
					pow = 0;
					m_left.set(DoubleSolenoid.Value.kForward);
					m_right.set(DoubleSolenoid.Value.kForward);
					Timer.delay(NerdyResources.Constants.pause);
					m_autoStackState = 4;
					break;
				case 4:
					pow = m_pid.setDesired(heights[2]).calculate(m_encode);
					if(Math.abs(m_encode - heights[2])<.1)	{
						m_autoStackState = -1;
						manual();
						m_pid.reset();
					}
					break;
				default:
					manual();
					break;
				}
			}	else if(m_state == State.kCommand)	{
				SmartDashboard.putBoolean("AutoStacking?", true);
				pow = m_pid.setDesired(m_desired).calculate(m_encode);
				if(Math.abs(m_encode - m_desired)<.1)	{
					m_pid.reset();
				}
			}
		}
		
		if(m_state == State.kManual)	{
			SmartDashboard.putBoolean("Carriage", m_open);
			if(m_open)	{
				m_left.set(DoubleSolenoid.Value.kForward);
				m_right.set(DoubleSolenoid.Value.kForward);
			}	else	{
				m_left.set(DoubleSolenoid.Value.kReverse);
				m_right.set(DoubleSolenoid.Value.kReverse);
			}
			
			if(m_clawOpen)	{
				m_claw.set(DoubleSolenoid.Value.kForward);
			}	else	{
				m_claw.set(DoubleSolenoid.Value.kReverse);
			}
		}
		
		m_lift.set((m_joy.getThrottle()+1)/2*NerdyMath.threshold(pow, -1, 1));
	}
}