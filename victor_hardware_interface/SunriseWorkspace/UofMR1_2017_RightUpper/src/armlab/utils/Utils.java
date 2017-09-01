package armlab.utils;

import java.util.concurrent.TimeUnit;

public class Utils
{
	public static void wait(final int milliseconds)
	{
		try
		{
		    TimeUnit.MILLISECONDS.sleep(milliseconds);
		}
		catch (InterruptedException e)
		{}
	}
	
	public static double getUTCTimeAsDouble()
	{
		return (double) System.currentTimeMillis() / 1000.0;
	}
}
