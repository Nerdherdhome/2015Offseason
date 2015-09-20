package org.camsrobotics.frc.subsystems;

import org.camsrobotics.frc.util.NerdyDrivebase;
import org.camsrobotics.frc.util.NerdyJoystick;
import org.camsrobotics.frc.util.NerdyMath;
import org.camsrobotics.frc.util.NerdyStates;

import com.kauailabs.nav6.frc.IMUAdvanced;

/**
 * Drive system for the robot. Drives a Mecanum base in both robot centric and field centric modes.
 * 
 * @author Wesley
 *
 */

public class NerdyDrive {
	/**
	 * Drive states
	 * 
	 * @author Wesley
	 *
	 */
	public static class State	{
        public final int state;
        private static final int kOff_val = -1;
        private static final int kRobotCentric_val = 0;
        private static final int kFieldCentric_val = 1;
        
        public static final State kOff = new State(kOff_val);
        public static final State kRobotCentric = new State(kRobotCentric_val);
        public static final State kFieldCentric = new State(kFieldCentric_val);

        private State(int state) {
            this.state = state;
        }
        
        public String toString()	{
        	switch(state)	{
        	case 0:
        		return "Off";
			case 1:
        		return "Robot Centric Mode";
			case 2:
				return "Field Centric Mode";
        	}
        	// Should never happen
			return null;
        }
	}
	
	private NerdyJoystick m_leftJoy, m_rightJoy;
	private IMUAdvanced m_imu;
	private NerdyDrivebase m_drivebase;
	private State m_state;
	private double m_fl, m_fr, m_bl, m_br;
	
	/**
	 * Default constructor
	 * 
	 * @param leftJoy
	 * @param rightJoy
	 * @param imu
	 * @param drivebase
	 */
	public NerdyDrive(NerdyJoystick leftJoy, NerdyJoystick rightJoy, IMUAdvanced imu, NerdyDrivebase drivebase){
		m_leftJoy = leftJoy;
		m_rightJoy = rightJoy;
		m_imu = imu;
		m_drivebase = drivebase;

		m_state = State.kFieldCentric;
		NerdyStates.put("Drive", m_state);
	}
	
	/**
	 * Disables the robot
	 */
	public void disable()	{
		m_state = State.kOff;
		m_drivebase.disable();
		NerdyStates.put("Drive", m_state);
	}
	
	/**
	 * Enables the robot. Default state: Field Centric
	 */
	public void enable()	{
		m_state = State.kFieldCentric;
		m_drivebase.enable();
		NerdyStates.put("Drive", m_state);
	}
	
	/**
	 * Drives the robot in Robot Centric mode
	 */
	public void robotCentric()	{
		m_state = State.kRobotCentric;
		m_drivebase.enable();
		NerdyStates.put("Drive", m_state);
	}
	
	/**
	 * Drives the robot in Field Centric mode
	 */
	public void fieldCentric()	{
		m_state = State.kFieldCentric;
		m_drivebase.enable();
		NerdyStates.put("Drive", m_state);
	}
	
	/**
	 * Calibrates the IMU
	 */
	public void calibrate()	{
		m_imu.zeroYaw();
	}
	
	// Calculation functions
	/**
	 * Calculates the RobotCentric function
	 */
	private void driveRobotCentric()	{
		double strafe = m_leftJoy.getX();
		double forward = m_leftJoy.getTrueY();
		double rotate = m_rightJoy.getX();
		
		m_fl = (forward + strafe) + rotate/2;
        m_fr = (-forward + strafe) + rotate/2;
        m_bl = (forward - strafe) + rotate/2;
        m_br = (-forward - strafe) + rotate/2;
	}
	
	/**
	 * Calculates the Field Centric function
	 */
	private void driveFieldCentric()    {
		double leftX = m_leftJoy.getX();
		double leftY = m_leftJoy.getTrueY();
		double rightX = m_rightJoy.getX();
		
		double gyroAngle = -m_imu.getYaw();
        double gyroAngleRads = NerdyMath.degToRads(gyroAngle);
        
        double desiredAngle = m_leftJoy.getAngleRad();
        double relativeAngle = (-(gyroAngleRads) + (desiredAngle) + (Math.PI/2)) % (2*Math.PI);
        
        double forward = Math.sin(relativeAngle);
        double strafe = Math.cos(relativeAngle);
        
        double unscaledJoy[] = {Math.sin(desiredAngle), Math.cos(desiredAngle)};
        double maxJoy[] = NerdyMath.normalize(unscaledJoy, true);
        double scalar = NerdyMath.threshold(((leftY * leftY) + (leftX * leftX)) / ((maxJoy[0] * maxJoy[0]) + (maxJoy[1] * maxJoy[1])), -1, 1);
        double rotate = rightX/2;
        double ftLeft = (forward + strafe)*scalar + rotate;
        double ftRight = (-forward + strafe)*scalar + rotate;
        double bkLeft = (forward - strafe)*scalar + rotate;
        double bkRight = (-forward - strafe)*scalar + rotate;
        double unnormalizedValues[] = {ftLeft, ftRight, bkLeft, bkRight};
        double output[] = NerdyMath.normalize(unnormalizedValues, false);
        
        m_fl	= output[0];
        m_fr	= output[1];
        m_bl	= output[2];
        m_br	= output[3];
    }
	
	/**
	 * Drives the robot. Run this every iteration
	 */
	public void run()	{
		if(m_state == State.kOff)	{
			m_fl = 0;
			m_fr = 0;
			m_bl = 0;
			m_br = 0;
		}	else if(m_state == State.kRobotCentric)	{
			driveRobotCentric();
		}	else if(m_state == State.kFieldCentric)	{
			driveFieldCentric();
		}
		
		m_drivebase.drive(m_fl, m_fr, m_bl, m_br);
	}
}
