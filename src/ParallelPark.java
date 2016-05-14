import lejos.nxt.Button;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;

public class ParallelPark {
	public static final int WHEEL_DIAMETER = 56;
	public static final int ROBOT_LENGTH = 180;
	public static final int DISTANCE_TO_SIDEWALK = 20;
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		UltrasonicSensor sens = new UltrasonicSensor(SensorPort.S1);
		sens.continuous();
		
		DifferentialPilot pilot = new DifferentialPilot(56, 120, Motor.A, Motor.C);
		pilot.setTravelSpeed(90);
		pilot.setRotateSpeed(20);
		System.out.println(sens.getDistance());
		double distance = Integer.MIN_VALUE;
		
		int error = 10;
		boolean found_spot = false;
		while(!canFit(distance)){
			Motor.A.resetTachoCount();
			Motor.C.resetTachoCount();
			pilot.forward();
			//closest thing is sidewalk
			if(sens.getDistance() >= DISTANCE_TO_SIDEWALK){
				while (sens.getDistance() >= DISTANCE_TO_SIDEWALK) 
					System.out.println(sens.getDistance());
				
				//if(sens.getDistance() > DISTANCE_TO_SIDEWALK)
				pilot.stop();
				distance = convertTachosToMilli(Motor.A.getTachoCount());
			}
			//theres an obstacle
			else{
				while (sens.getDistance() < DISTANCE_TO_SIDEWALK) 
					System.out.println(sens.getDistance());
			 	
				pilot.stop();
			}
		}
		pilot.stop();
		System.out.println("length : "+distance);
		System.out.println(canFit(distance));
		if (canFit(distance))
			startParking(pilot);
		Button.waitForAnyPress();
	}
	
	public static void startParking(DifferentialPilot pilot) {
		//pilot.travel(-120);
		//pilot.rotate(45);
		pilot.travel(-90);
		pilot.travelArc(-120, -100);
		pilot.travel(-130);
		pilot.travelArc(100, -75);
		//pilot.travelArc(-100, 25);
		pilot.travel(50);
		//pilot.rotate(-45);
		//pilot.travel(30);
	}
	public static double convertTachosToMilli(int tachos){
		return (tachos/360)*Math.PI*WHEEL_DIAMETER;
	}
	
	public static boolean canFit(double length) {
		return length > ROBOT_LENGTH;
	}
}
