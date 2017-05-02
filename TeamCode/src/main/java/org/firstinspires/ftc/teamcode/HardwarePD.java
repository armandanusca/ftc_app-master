package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.robocol.RobocolParsable.MsgType.GAMEPAD;
import static org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot.MID_SERVO;
import static org.firstinspires.ftc.teamcode.R.layout.servo;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePD
{
    /* Public OpMode members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public DcMotor  catchMotor  = null;
    public DcMotor  throwMotor  = null;
    public DcMotor  liftMotor  = null;
    public Servo fixServo = null;
    public Servo releaseServo = null;
    public Servo loadServo = null;
    //public Servo LClawServo = null;
    //public Servo RClawServo = null;
    public ColorSensor Color= null;
    //public GyroSensor Giro=null;
    public Servo hitBeacon = null;
    public OpticalDistanceSensor Distanta = null;
    //public CompassSensor Dirt=null;
    /* local OpMode members. */
    HardwareMap map =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePD(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap aMap) {
        // Save reference to Hardware map
        map = aMap;
        // Define and Initialize Motors
        leftMotor   = map.dcMotor.get("left_drive");
        rightMotor  = map.dcMotor.get("right_drive");
        catchMotor  = map.dcMotor.get("catch");
        throwMotor  = map.dcMotor.get("throw");
        liftMotor  = map.dcMotor.get("lift");
        loadServo = map.servo.get("load");
        hitBeacon = map.servo.get("beacon");
        fixServo = map.servo.get("fix");
        releaseServo = map.servo.get("release");
        Distanta = map.opticalDistanceSensor.get("distanta");
        //Dirt = map.compassSensor.get("compass");
        ///RClawServo = map.servo.get("right_claw");
        ///LClawServo.setPosition(Servo.MIN_POSITION);
        ///RClawServo.setPosition(Servo.MIN_POSITION);
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        catchMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        throwMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        ///liftMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        catchMotor.setPower(0);
        throwMotor.setPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        catchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        throwMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ///liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Define and initialize ALL installed servos.
        /*leftClaw = hwMap.servo.get("left_hand");
        rightClaw = hwMap.servo.get("right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);*/
        //Giro=map.gyroSensor.get("Giro");
        Color=map.colorSensor.get("culoare");
        fixServo.setPosition(1);
        releaseServo.setPosition(Servo.MAX_POSITION);
        loadServo.setPosition(0.5);
        hitBeacon.setPosition(Servo.MAX_POSITION);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
}

