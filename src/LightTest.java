import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.LightSensor;
import lejos.nxt.SensorPort;
import lejos.util.Delay;

public class LightTest {
	public static void main(String[] args) {
		LightSensor sensor = new LightSensor(SensorPort.S3);
		sensor.setFloodlight(true );
		while (true) {
			System.out.println(sensor.getNormalizedLightValue());
			Delay.msDelay(100);
		}
		
	}
	
}

