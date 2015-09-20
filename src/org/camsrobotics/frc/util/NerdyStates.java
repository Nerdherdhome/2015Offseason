package org.camsrobotics.frc.util;

import java.util.HashMap;

/**
 * Static class to hold robot states
 * 
 * @author Wesley
 *
 */
public class NerdyStates {
	private static HashMap<String, Object> m_states = new HashMap<String, Object>();
	
	/**
	 * Adds a value
	 * 
	 * @param key
	 * @param value
	 */
	public static void put(String key, Object value)	{
		m_states.put(key, value);
	}
	
	/**
	 * Gets a value
	 * 
	 * @param key The value's key
	 * @return The value
	 */
	public static Object get(String key)	{
		return m_states.get(key);
	}
	
	/**
	 * Gets the key set
	 * 
	 * @return A string array
	 */
	public static String[] keySet()	{
		return (String[]) m_states.keySet().toArray();
	}
}
