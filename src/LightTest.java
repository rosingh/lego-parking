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
	private static int DISTANCE_TO_SIDEWALK = 20;
	
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
		boolean detectedCar = false;
		while (!canFit(distance)) {
			int current_value = sensor.getNormalizedLightValue();
			System.out.println(current_value + " vs " + white_value); 
			
			//detected black line
			if (current_value <= white_value - threshold) {
				pilot.forward();
				//this is the first time black line is detected
				if (!sawLine) {
					//continue until black line goes away, this is to avoid problems with thick lines
					while (sensor.getNormalizedLightValue() <= white_value - threshold);
					sawLine = true;
					//restart tachos -> start of a new parking spaces
					pilot.stop();
					Motor.A.resetTachoCount();
					Motor.C.resetTachoCount();
				}
				//this isn't the first black line detected
				else {
					pilot.stop();
					//measure out the length of the parking space
					distance = convertTachosToMilli(Motor.A.getTachoCount());
					
					//try to park if it can fit
					System.out.println("DETECTED CAR? "+detectedCar);
					if(!detectedCar && canFit(distance)){
						linePark(pilot, distance);
						break;
					}
					
					//cant fit-> go forward until passes black line
					pilot.forward();
					while(sensor.getNormalizedLightValue() <= white_value - threshold);
					
					//reset tachos - this is beginning of next parkig spaces
					pilot.stop();
					Motor.A.resetTachoCount();
					Motor.C.resetTachoCount();
					
					detectedCar = false;
				}
			}
			//white space detected
			else{
				pilot.forward();
				//keep going until black line is detected
				while(sensor.getNormalizedLightValue() > white_value - threshold){
					if(SIDE_SENSOR.getDistance() < DISTANCE_TO_SIDEWALK)
						detectedCar = true;
				};
				pilot.stop();
			}
		}
		System.out.println("Distance : " + distance);
		Button.waitForAnyPress();
	}
	
	public static boolean canFit(double width) {
		return width > ROBOT_WIDTH;
	}
	
	public static double convertTachosToMilli(int tachos){
		System.out.println("TACOS : "+tachos);
		return (tachos/360.0)*Math.PI*WHEEL_DIAMETER;
	}
	
	public static void linePark(DifferentialPilot pilot, double parkingWidth){
		int ultrasonicError = 25;
		//pilot.travel(width*-3/4);
		pilot.travel(-ROBOT_LENGTH/2);
		//pilot.arc(parkingWidth / 2, -90);
		pilot.arc((parkingWidth+ROBOT_WIDTH/2)/2, -90);
		FRONT_SENSOR.ping();
		Delay.msDelay(50);
		int curbDistance = FRONT_SENSOR.getDistance();
		pilot.travel(curbDistance*10-ultrasonicError);
		System.out.println("DISTANCE TO CURB : "+curbDistance);
	}
}

