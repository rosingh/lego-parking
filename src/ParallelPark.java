import lejos.nxt.Button;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.util.Delay;

public class ParallelPark {
	public static final int WHEEL_DIAMETER = 56;
	private static int ROBOT_WIDTH = 166;
	private static int ROBOT_LENGTH = 204;
	public static final int DISTANCE_TO_SIDEWALK = 20;
	public static final int ACCEPTABLE_DISTANCE_FROM_CURB = 8;
	
	private static final UltrasonicSensor FRONT_SENSOR = new UltrasonicSensor(SensorPort.S2);
	private static final UltrasonicSensor SIDE_SENSOR = new UltrasonicSensor(SensorPort.S1);
	
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		SIDE_SENSOR.continuous();
		FRONT_SENSOR.ping();
		
		DifferentialPilot pilot = new DifferentialPilot(56, 120, Motor.A, Motor.C);
		pilot.setTravelSpeed(60);
		pilot.setRotateSpeed(15);
		System.out.println(SIDE_SENSOR.getDistance());
		double distance = Integer.MIN_VALUE;
		
		int error = 5;
		boolean found_spot = false;
		while(!canFit(distance)){
			Motor.A.resetTachoCount();
			Motor.C.resetTachoCount();
			pilot.forward();
			//closest thing is sidewalk
			if(SIDE_SENSOR.getDistance() >= DISTANCE_TO_SIDEWALK - error){
				// continue until an obstacle is reached
				while (SIDE_SENSOR.getDistance() >= DISTANCE_TO_SIDEWALK - error);
				
				pilot.stop();
				distance = convertTachosToMilli(Motor.A.getTachoCount());
			}
			//theres an obstacle - this is not a parkig sp
			else{
				while(SIDE_SENSOR.getDistance() < DISTANCE_TO_SIDEWALK);
			 	
				pilot.stop();
			}
		}
		pilot.stop();
		System.out.println("length : "+distance);
		System.out.println(canFit(distance));
		if (canFit(distance)){
			parallelParkBack(pilot, distance, Integer.MAX_VALUE);
		}
		Button.waitForAnyPress();
	}
	
	public static void parallelParkBack(DifferentialPilot pilot, double parkingWidth, int prevCurbDist) {
		int ultrasonicError = 25;
		int yDistance = SIDE_SENSOR.getDistance() * 10;
		double radius;
		//if this was the first time we're trying to back in, clear the back, and account for it in radius
		if(prevCurbDist == Integer.MAX_VALUE){
			pilot.travel(-ROBOT_LENGTH / 2);
			radius = Math.sqrt(Math.pow(yDistance, 2) + Math.pow((parkingWidth-ROBOT_LENGTH)/2, 2)) - 25;
		}
		else
			radius = Math.sqrt(Math.pow(yDistance, 2) + Math.pow(parkingWidth/2, 2)) - 25;
		double angle= Math.atan2(parkingWidth/2, yDistance) * 180 / Math.PI + 10;
		if(angle > 45)
			angle = 45;
		radius = radius/2;
		System.out.println("RADIUS : "+radius + " ANGLE "+angle);
		pilot.arc(-radius, angle);
		
		//moves the robot diagonally to the back
		SIDE_SENSOR.ping();
		Delay.msDelay(50);
		//double diagDistance = SIDE_SENSOR.getDistance() * 10 - radius - ultrasonicError;
		double diagDistance = SIDE_SENSOR.getDistance() * 10 /Math.atan(angle);
		System.out.println("CLOSING IN : " + diagDistance);
		if(SIDE_SENSOR.getDistance() != 255 && diagDistance > 0)
			pilot.travel(-diagDistance);
		
		//straightens robot
		pilot.arc(radius/2, -angle);
		
		//measure distance from curb
		SIDE_SENSOR.ping();
		Delay.msDelay(50);
		int curbDistance = SIDE_SENSOR.getDistance();
		System.out.println("CURB1 " + curbDistance);
		//should we go closer?
		if(curbDistance > ACCEPTABLE_DISTANCE_FROM_CURB && curbDistance < prevCurbDist){
			//back up the robot as far as it can
			FRONT_SENSOR.ping();
			Delay.msDelay(50);
			double distanceToFront = FRONT_SENSOR.getDistance()*10 - ultrasonicError;
			if(distanceToFront > parkingWidth - ROBOT_WIDTH)
				distanceToFront = parkingWidth - ROBOT_WIDTH;
			//pilot.travel(distanceToFront);
			double distanceBehind = (parkingWidth-distanceToFront-ROBOT_LENGTH/2);
			//System.out.println("BEHIND ME : "+distanceBehind);
			if(distanceBehind > 0){
				pilot.travel(-distanceBehind);
			}
			
			if(prevCurbDist == Integer.MAX_VALUE)
				parallelParkForward(pilot, distanceToFront, curbDistance);
			else
				parallelParkForward(pilot, parkingWidth, curbDistance);
		}
	}
	
	public static void parallelParkForward(DifferentialPilot pilot, double parkingWidth, int prevCurbDist) {
		int ultrasonicError = 25;
		
		//rotates the robot, so that it goes closer to the curb
		int yDistance = SIDE_SENSOR.getDistance() * 10;
		//double radius = ROBOT_LENGTH/Math.tan(37*Math.PI/180);
		double radius = Math.sqrt(Math.pow(yDistance, 2) + Math.pow((parkingWidth)/2, 2)) - 25;	
		double angle= Math.atan2(parkingWidth/2, yDistance) * 180 / Math.PI;
		System.out.println("ORIGINAL ANGLE : "+angle);
		if(angle > 45)
			angle = 45;
		radius = radius/2;
		//System.out.println("RADIUS : "+radius + " ANGLE "+angle);
		pilot.arc(-radius, -angle);
		
		//moves the robot diagonally to the front
		FRONT_SENSOR.ping();
		Delay.msDelay(50);
		double headSpace = FRONT_SENSOR.getDistance() * 10;
		if(FRONT_SENSOR.getDistance() != 255 && headSpace > parkingWidth)
			headSpace = parkingWidth;
		double diagDistance = headSpace - radius - ultrasonicError - 20;
		//System.out.println("CLOSING IN : " + diagDistance);
		if(diagDistance > 0)
			pilot.travel(diagDistance);
		
		//straigtens out the robot
		pilot.arc(radius/2, angle);
		
		//gets distance from curb
		SIDE_SENSOR.ping();
		Delay.msDelay(50);
		int curbDistance = SIDE_SENSOR.getDistance();
		System.out.println("CURB2 " + curbDistance);
		
		if(curbDistance > ACCEPTABLE_DISTANCE_FROM_CURB && curbDistance < prevCurbDist){
			//moves the robot to the very front
			FRONT_SENSOR.ping();
			Delay.msDelay(50);
			if(FRONT_SENSOR.getDistance() != 255){
				double distanceToFront = FRONT_SENSOR.getDistance()*10 - ultrasonicError;
				if(distanceToFront > parkingWidth - ROBOT_WIDTH)
					distanceToFront = parkingWidth - ROBOT_WIDTH;
				pilot.travel(distanceToFront);
			}
			//now try to get closer :)
			parallelParkBack(pilot, parkingWidth, curbDistance);
		}
	}
	
	public static double convertTachosToMilli(int tachos){
		return (tachos/360.0)*Math.PI*WHEEL_DIAMETER;
	}
	
	public static boolean canFit(double length) {
		//the spot must be at least 304mm for the robot to fit. 
		return length >= ROBOT_LENGTH * 1.5;
	}
}
