package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * This is NOT an opmode.
 *
 * This class is used to define all the specific hardware for our robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 *
 */

public class AtomicGearsHW {

    /** Public OpMode Members **/

    public DcMotor FrontLeft = null;
    public DcMotor FrontRight = null;
    public DcMotor BackLeft = null;
    public DcMotor BackRight = null;
    public DcMotor GylphIntakeRight = null;
    public DcMotor GylphIntakeLeft = null;
    public DcMotor Gylpht = null;
    public DcMotor LED = null;
    public CRServo LeftBeltIntakeServo = null;
    public CRServo RightBeltIntakeSevo = null;
    public CRServo Elbow = null;
    public CRServo Wrist = null;
    public CRServo Flippy = null;
    public Servo JewelPan = null;
    public Servo JewelTilt = null;
    public Servo ClawRotation = null;
    public Servo Claw = null;
    public Servo Base = null;
    public Servo Shoulder = null;
    public DigitalChannel top = null;
    public DigitalChannel bottom = null;
    public AnalogInput pot = null;

    public static final double COUNTS_PER_MOTOR_REV =  537.6;    // eg: Andy Mark Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 1.333333333333333;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    /** Local OpMode Members. **/

    HardwareMap hwMap = null;

    /** Constructor **/

    public AtomicGearsHW() {
    }

    /** Initialize standard Hardware interfaces **/

    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FrontLeft = hwMap.get(DcMotor.class,"fl");
        FrontRight = hwMap.get(DcMotor.class,"fr");
        BackLeft = hwMap.get(DcMotor.class,"bl");
        BackRight = hwMap.get(DcMotor.class,"br");
        GylphIntakeLeft = hwMap.get(DcMotor.class, "gil");
        GylphIntakeRight = hwMap.get(DcMotor.class, "gir");
        Gylpht = hwMap.get(DcMotor.class,"gl");
        LeftBeltIntakeServo = hwMap.get(CRServo.class, "lbs");
        RightBeltIntakeSevo = hwMap.get(CRServo.class, "rbs");
        Base = hwMap.get(Servo.class, "b");
        Shoulder = hwMap.get(Servo.class, "s");
        Elbow = hwMap.get(CRServo.class, "e");
        Wrist = hwMap.get(CRServo.class, "w");
        ClawRotation = hwMap.get(Servo.class, "cr");
        Claw = hwMap.get(Servo.class, "c");
        JewelTilt = hwMap.get(Servo.class, "jt");
        JewelPan = hwMap.get(Servo.class, "jp");
        Flippy = hwMap.get(CRServo.class,"f");
        top = hwMap.get(DigitalChannel.class,"lt");
        bottom = hwMap.get(DigitalChannel.class,"lb");
        pot = hwMap.get(AnalogInput.class,"p");
        LED = hwMap.get(DcMotor.class,"led");






        // Set all motors to zero power
        LED.setPower(1);
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
        GylphIntakeLeft.setPower(0);
        GylphIntakeRight.setPower(0);
       Gylpht.setPower(0);
       RightBeltIntakeSevo.setPower(0);
       LeftBeltIntakeServo.setPower(0);
       Elbow.setPower(0);
       Wrist.setPower(0);

        // Set all motors to zero power
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GylphIntakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GylphIntakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       Gylpht.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


      top.setMode(DigitalChannel.Mode.INPUT);
      bottom.setMode(DigitalChannel.Mode.INPUT);


    }

    /**
     This method scales the joystick input so for low joystick values, the
     scaled value is less than linear.  This is to make it easier to drive
     the robot more precisely at slower speeds.
     */

    public double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    public void Forward(double speed, double distance){
        FrontLeft.setTargetPosition((int) (-distance* COUNTS_PER_INCH));
        FrontRight.setTargetPosition((int) (distance* COUNTS_PER_INCH));
        BackLeft.setTargetPosition((int) (-distance* COUNTS_PER_INCH));
        BackRight.setTargetPosition((int) (distance* COUNTS_PER_INCH));
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setPower(-speed);
        FrontRight.setPower(speed);
        BackLeft.setPower(-speed);
        BackRight.setPower(speed);

    }

    public void Reverse(double speed, double distance){
        FrontLeft.setTargetPosition((int) (distance* COUNTS_PER_INCH));
        FrontRight.setTargetPosition((int) (-distance* COUNTS_PER_INCH));
        BackLeft.setTargetPosition((int) (distance* COUNTS_PER_INCH));
        BackRight.setTargetPosition((int) (-distance* COUNTS_PER_INCH));
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setPower(speed);
        FrontRight.setPower(-speed);
        BackLeft.setPower(speed);
        BackRight.setPower(-speed);
    }

    public void Right(double speed, double distance){
        FrontLeft.setTargetPosition((int) (-distance * 207 / 179 * COUNTS_PER_INCH));
        FrontRight.setTargetPosition((int) (-distance * 207 / 179 * COUNTS_PER_INCH));
        BackLeft.setTargetPosition((int) (distance * 207 / 179 * COUNTS_PER_INCH));
        BackRight.setTargetPosition((int) (distance * 207 / 179 * COUNTS_PER_INCH));
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setPower(-speed);
        FrontRight.setPower(-speed);
        BackLeft.setPower(speed);
        BackRight.setPower(speed);
    }

    public void Left(double speed, double distance){
        FrontLeft.setTargetPosition((int) (distance * 207 / 179 * COUNTS_PER_INCH));
        FrontRight.setTargetPosition((int) (distance * 207 / 179 * COUNTS_PER_INCH));
        BackLeft.setTargetPosition((int) (-distance * 207 / 179 * COUNTS_PER_INCH));
        BackRight.setTargetPosition((int) (-distance * 207 / 179 * COUNTS_PER_INCH));
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setPower(speed);
        FrontRight.setPower(speed);
        BackLeft.setPower(-speed);
        BackRight.setPower(-speed);
    }

    public void Turn(double speed){

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setPower(-speed);
        FrontRight.setPower(-speed);
        BackLeft.setPower(-speed);
        BackRight.setPower(-speed);

    }



    public void TurnAbsoulte(double target, double heading){

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double Error   = heading - target;
        double Kp = 0.015;
        double LeftPower;
        double RightPower;

        if ((Math.abs(Error)) > 2 ){
            LeftPower = Error * Kp;
            RightPower = -Error * Kp;
            Range.clip(LeftPower,-1,1);
            Range.clip(RightPower,-1,1);

            FrontLeft.setPower(-LeftPower);
            FrontRight.setPower(RightPower);
            BackLeft.setPower(-LeftPower);
            BackRight.setPower(RightPower);
        }
        else {
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
        }

    }

    /**
    PID Proportional loop for multiple range sensors on the bot
     Selet your TARGET in CM
     Apply your RANGE in your OpMode
     Select your direction 0=Right 1=Left 2=Back
    **/

    public void Wall(double target, double range, double direction){

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double Error   = range - target;
        double Kp = 0.02;
        double Strafe;

        if ((Math.abs(Error)) > 1 ){
            Strafe = Error * Kp;

            if (direction == 0){
            FrontLeft.setPower(-Strafe);
            FrontRight.setPower(-Strafe);
            BackLeft.setPower(Strafe);
            BackRight.setPower(Strafe);}

            if (direction == 1){
                FrontLeft.setPower(Strafe);
                FrontRight.setPower(Strafe);
                BackLeft.setPower(-Strafe);
                BackRight.setPower(-Strafe);
            }

            if (direction == 2){
                FrontLeft.setPower(Strafe*0.3);
                FrontRight.setPower(-Strafe*0.3);
                BackLeft.setPower(Strafe*0.3);
                BackRight.setPower(-Strafe*0.3);}



        }
        else {
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
        }

    }

    public void DriveWithPower(double speed){
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setPower(-speed);
        FrontRight.setPower(speed);
        BackLeft.setPower(-speed);
        BackRight.setPower(speed);
    }

    public void StrafeWithPower(double speed){

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setPower(-speed);
        FrontRight.setPower(-speed);
        BackLeft.setPower(speed);
        BackRight.setPower(speed);
    }

    /**
     Stops everything and resets encoders
     */

    public void Kill(){
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     Checks to see if the motors are running
     and if they're not it will return true
     */

    public boolean IsBusy(){
        if (!FrontLeft.isBusy() || !BackRight.isBusy())
        {
            return (true);
        } else return (false);
    }

    public boolean DriveDone(double distance){
        if (Math.abs(FrontLeft.getCurrentPosition() / COUNTS_PER_INCH) >= distance
                || Math.abs(BackRight.getCurrentPosition() / COUNTS_PER_INCH) >= distance)
        {
            return (true);
        } else return (false);
    }

    public boolean StrafeDone(double distance){
        if (Math.abs(FrontLeft.getCurrentPosition() / COUNTS_PER_INCH * 207 /179) >= distance
                || Math.abs(BackRight.getCurrentPosition() / COUNTS_PER_INCH * 207 / 179 ) >= distance)
        {
            return (true);
        } else return (false);
    }

}



