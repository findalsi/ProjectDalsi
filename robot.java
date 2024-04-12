import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

/**
 * This class implements a line follower robot that can avoid obstacles.
 */
public class LineFollowerRobot {
    static EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
    static EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.B);
    static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
    static EV3UltrasonicSensor distanceSensor = new EV3UltrasonicSensor(SensorPort.S1);
    static long startTime; // Variable to store start time
    static int obstacleCount = 0; // Variable to count obstacles

    /**
     * The main method of the LineFollowerAvoidingObstacles class.
     * Starts the robot and controls its behavior.
     * @param args The command-line arguments (not used)
     */
    public static void main(String[] args) {
        // Initialization of sensors and motors
        SampleProvider colorProvider = colorSensor.getRedMode();
        SampleProvider distanceProvider = distanceSensor.getDistanceMode();
        float[] colorSample = new float[colorProvider.sampleSize()];
        float[] distanceSample = new float[distanceProvider.sampleSize()];

        // Set initial motor speeds
        leftMotor.setSpeed(200);
        rightMotor.setSpeed(200);

        boolean obstacleDetected = false;
        startTime = System.currentTimeMillis(); // Record start time

        // Main control loop
        while (!Button.ESCAPE.isDown()) {
            colorProvider.fetchSample(colorSample, 0);
            distanceProvider.fetchSample(distanceSample, 0);

            float colorValue = colorSample[0] * 100; // Scale to 0-100 range
            float distanceValue = distanceSample[0] * 100; // Convert to centimeters

            // Line following behavior
            if (!obstacleDetected) {
                if (colorValue < 13) { // Adjust the threshold as needed
                    leftMotor.forward();
                    rightMotor.forward();
                } else {
                    leftMotor.forward();
                    rightMotor.backward();
                }
            }

            // Obstacle detection and avoiding obstacle
            if (!obstacleDetected && distanceValue < 10) { // Obstacle detected within 10 cm
                obstacleDetected = true;
                leftMotor.stop();
                rightMotor.stop();
                obstacleCount++; // Increment obstacle count
                if (obstacleCount == 2) { // Stop when second obstacle is detected
                    long elapsedTime = System.currentTimeMillis() - startTime;
                    LCD.drawString("Elapsed Time: " + elapsedTime / 1000 + " seconds", 0, 0);
                    playMusic(); // Play music before stopping
                    Sound.beepSequence(); // Play a beep sound
                    Button.waitForAnyPress();
                    break; // Exit the loop after displaying elapsed time
                }
                turnRight90();
                moveForward();
                turnLeft90();
                moveForward();
                turnLeft90();
                moveForward();
                obstacleDetected = false; // Resume line following after obstacle avoidance
            }

            // Display elapsed time on LCD screen
            long elapsedTime = System.currentTimeMillis() - startTime;
            LCD.drawString("Time: " + elapsedTime / 1000 + " s", 0, 4);

            // Add a small delay to reduce CPU load
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        // Close sensors and motors after ESCAPE button press
        colorSensor.close();
        distanceSensor.close();
        leftMotor.close();
        rightMotor.close();
    }

    /**
     * Turns the robot right by approximately 90 degrees.
     */
    private static void turnRight90() {
        // Turn right 90 degrees (adjust motor speeds and duration as needed)
        leftMotor.setSpeed(150); // Adjust motor speed for smoother turning
        rightMotor.setSpeed(150); // Adjust motor speed for smoother turning

        // Rotate right motor backward and left motor forward to turn right
        leftMotor.forward();
        rightMotor.backward();

        // Wait for a duration to achieve a turn (adjust as needed)
        try {
            Thread.sleep(900); // Adjust duration for smoother turning
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * Moves the robot forward.
     */
    private static void moveForward() {
        // Move forward after turning right or left
        leftMotor.setSpeed(200); // Adjust motor speed as needed
        rightMotor.setSpeed(200); // Adjust motor speed as needed

        // Both motors move forward
        leftMotor.forward();
        rightMotor.forward();

        // Wait for a duration to move forward (adjust as needed)
        try {
            Thread.sleep(2500); // Adjust duration as needed for forward movement
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * Turns the robot left by approximately 90 degrees.
     */
    private static void turnLeft90() {
        // Turn left 90 degrees (adjust motor speeds and duration as needed)
        leftMotor.setSpeed(150); // Adjust motor speed for smoother turning
        rightMotor.setSpeed(150); // Adjust motor speed for smoother turning

        // Rotate left motor backward and right motor forward to turn left
        leftMotor.backward();
        rightMotor.forward();

        // Wait for a duration to achieve a turn (adjust as needed)
        try {
            Thread.sleep(900); // Adjust duration for smoother turning
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    
    /**
     * Plays a simple melody.
     */
    private static void playMusic() {
        int[] notes = { 440, 494, 523, 587, 659, 698, 784, 880 }; // Frequencies of notes (A4 - A5)
        int[] durations = { 200, 200, 200, 200, 200, 200, 200, 200 }; // Durations of notes (milliseconds)

        // Play the melody
        for (int i = 0; i < notes.length; i++) {
            Sound.playTone(notes[i], durations[i]);
            try {
                Thread.sleep(50); // Pause between notes
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}

