import lejos.nxt.Button;
import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.util.Delay;

/* NOTE: ALL DISTANCES ARE IN MM! */
public class LightTest {
	private static int ROBOT_WIDTH = 165;
	private static int ROBOT_LENGTH = 203;
	private static int WHEEL_DIAMETER = 56;
	
	private static final UltrasonicSensor FRONT_SENSOR = new UltrasonicSensor(SensorPort.S2);
	private static final UltrasonicSensor SIDE_SENSOR = new UltrasonicSensor(SensorPort.S1);
	
	public static void main(String[] args) {
		FRONT_SENSOR.ping();
		SIDE_SENSOR.ping();
		
		LightSensor sensor = new LightSensor(SensorPort.S3);
		sensor.setFloodlight(true);
		
		DifferentialPilot pilot = new DifferentialPilot(56, 120, Motor.A, Motor.C);
		pilot.setTravelSpeed(60);
		pilot.setRotateSpeed(15);
		
		// get initial light reading to account for ambient light
		Delay.msDelay(1000);
		int initVal = sensor.getNormalizedLightValue();
		int threshold = 10 ;
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
					linePark(pilot, distance);
					System.out.println("CAN FIT? "+canFit(distance));
					break;
				}
			}
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
	
	public static void linePark(DifferentialPilot pilot, double parkingWidth){
		//pilot.travel(width*-3/4);
		pilot.travel(-ROBOT_LENGTH/2);
		//pilot.arc(parkingWidth / 2, -90);
		pilot.arc((parkingWidth+ROBOT_WIDTH/2)/2, -90);
		FRONT_SENSOR.ping();
		Delay.msDelay(50);
		int curbDistance = FRONT_SENSOR.getDistance();
		pilot.travel(curbDistance*10-10);
		System.out.println("DISTANCE TO CURB : "+curbDistance);
	}
}

