import lejos.nxt.Button;
import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.util.Delay;

/* NOTE: ALL DISTANCES ARE IN MM!
 * 
 */
public class LightTest {
	private static int ROBOT_WIDTH = 200;
	private static int WHEEL_DIAMETER = 56;
	public static void main(String[] args) {
		LightSensor sensor = new LightSensor(SensorPort.S3);
		sensor.setFloodlight(true);
		
		DifferentialPilot pilot = new DifferentialPilot(56, 120, Motor.A, Motor.C);
		pilot.setTravelSpeed(90);
		pilot.setRotateSpeed(100);
		
		// get initial light reading to account for ambient light
		int initVal = sensor.getNormalizedLightValue();
		int threshold = 10;
		double distance = Integer.MIN_VALUE;
		boolean sawLine = false;
		int white_value = initVal;
		
		while (!canFit(distance)) {
			pilot.forward();
			int current_value = sensor.getNormalizedLightValue();
			System.out.println(current_value + " vs " + white_value); 
			if (current_value <= white_value - threshold) {
				if (!sawLine) {
					//continue until black line goes away, this is to avoid problems with thick lines
					while (sensor.getNormalizedLightValue() <= white_value - threshold) {
						//System.out.println(sensor.getNormalizedLightValue());
					}
					sawLine = true;
					pilot.stop();
					Motor.A.resetTachoCount();
					Motor.C.resetTachoCount();
				}
				else {
					pilot.stop();
					distance = convertTachosToMilli(Motor.A.getTachoCount());
					Motor.A.resetTachoCount();
					Motor.C.resetTachoCount();
					break;
				}
			}
			//else
				//white_value = current_value;
		}
		System.out.println("Distance : " + distance);
		Button.waitForAnyPress();
	}
	
	public static boolean canFit(double width) {
		return width > ROBOT_WIDTH;
	}
	
	public static double convertTachosToMilli(int tachos){
		System.out.println(tachos);
		return (tachos/360.0)*Math.PI*WHEEL_DIAMETER;
	}
	
}

