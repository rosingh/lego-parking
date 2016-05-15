import lejos.nxt.Button;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;

public class ParallelPark {
	public static final int WHEEL_DIAMETER = 56;
	private static int ROBOT_WIDTH = 165;
	private static int ROBOT_LENGTH = 203;
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
			startParking(pilot, distance, sens);
		Button.waitForAnyPress();
	}
	
	public static void startParking(DifferentialPilot pilot, double parkingWidth, UltrasonicSensor sensor) {
		pilot.travel(-ROBOT_LENGTH / 2);
		int yDistance = sensor.getDistance();
		pilot.arc(-ROBOT_WIDTH / 2, 45);
		//pilot.travel(-ROBOT_WIDTH / 2);
		pilot.travel(-yDistance*10/2);
		//pilot.arc(ROBOT_WIDTH / 2, -45);
		//radius = how much further we have left to go => initial parking width - backing up width - rotate to 45 deg (x) - backing up (x)
		pilot.arc((parkingWidth-ROBOT_LENGTH/2-ROBOT_WIDTH/2/Math.sqrt(2)-yDistance*5/Math.sqrt(2)), -45);
		
		wiggle(pilot, parkingWidth, sensor);
	}
	
	public static void wiggle(DifferentialPilot pilot, double parkingWidth, UltrasonicSensor sensor) {
		/*
		pilot.arc(-ROBOT_WIDTH, -23);
		pilot.arc(ROBOT_WIDTH / 2, 23);
		*/
		pilot.travel(parkingWidth - ROBOT_LENGTH);
		int yDistance = sensor.getDistance() * 10;
		System.out.println("YDISTANCE : "+yDistance);
		double angle = Math.asin((double)yDistance/(double)ROBOT_WIDTH*2)*180/Math.PI;
		System.out.println("ANGLE : " + angle);
		pilot.arc(-ROBOT_WIDTH / 2, angle);
		pilot.travel(-yDistance*10/2);
		//radius = how much further we have left to go => initial parking width - backing up width - rotate to 45 deg (x) - backing up (x)
		pilot.arc((parkingWidth-ROBOT_LENGTH-ROBOT_WIDTH/2/Math.sqrt(2)-yDistance*10/Math.sqrt(2)), -angle);
		
	}
	
	public static double convertTachosToMilli(int tachos){
		return (tachos/360.0)*Math.PI*WHEEL_DIAMETER;
	}
	
	public static boolean canFit(double length) {
		return length >= ROBOT_LENGTH;
	}
}
