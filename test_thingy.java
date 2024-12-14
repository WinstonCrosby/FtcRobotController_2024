package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;

@TeleOp(name = "TeleOp")
public class test_thingy extends OpMode {
    //name motors
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor ArmMotor;
    DcMotor ExtendArm;

    //Name servos
    CRServo grabber;
    Servo wrist;

    //Initialize desired value
    double desVal = 0;
    double wanVal = 0;

    //Initialize error
    double prevErr = 0;
    double prevExt = 0;

    double dsVal = 0;
    double prevEr = 0;
    //initialize
    @Override
    public void init() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ExtendArm = hardwareMap.get(DcMotor.class, "ExtendArm");
        grabber = hardwareMap.get(CRServo.class, "grabber");
        wrist = hardwareMap.get(Servo.class, "wrist");

        telemetry.addData("Hardware", "much gud");
    }

    @Override
    public void start()
    {

        //Set wrist position to startin value
        wrist.setPosition(0.39);

    }

    //loop start
    @Override
    public void loop () {

        //Take in directional inputs
        float y = gamepad1.left_stick_y/2;
        float x = gamepad1.left_stick_x/2;
        float t = gamepad1.right_stick_x/2;
        float ea = -gamepad2.right_stick_y/2;

        //Move this to Start() or Init()?
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //up and down, left right, and turning
        motorFL.setPower(y - x - t);
        motorFR.setPower(-y - x - t);
        motorBL.setPower(y + x - t);
        motorBR.setPower(-y + x - t);

        //This is incorrect
        if (ArmMotor.getCurrentPosition() >= -10){
            if (ArmMotor.getCurrentPosition() < 3300) {
            ArmMotor.setPower(ea);
            }
        }

        double mvVal = ArmMotor.getCurrentPosition();
        double error = dsVal - mvVal;

        if (ea != 0){
            dsVal = ArmMotor.getCurrentPosition();
        }
        if (gamepad2.a){
            ArmMotor.setPower(0);
            dsVal = ArmMotor.getCurrentPosition();
        }
        telemetry.addData("Arm Value", ArmMotor.getCurrentPosition());
        if (!gamepad2.a) {
            if (ea == 0) {
                double op = 0.0005;
                double od = 0.0005;

                double errod = error - prevEr;
                prevEr = error;
                double errop = error;

                double Go = (op * errop) + (od * errod);
                ArmMotor.setPower(Go);
                mvVal = ArmMotor.getCurrentPosition();

            }
        }


        //Arm up define arm values

        double upVal = 1526;
        double dwnVal = 0;
        double outVal = -1310;
        double inVal = -25;
        double lenVal = ExtendArm.getCurrentPosition();

        // If X button held, make grabber pickup
        if(gamepad2.x) {
            grabber.setPower(-1);
        }

        //If B button held, make grabber spit out
        else if(gamepad2.b){
            grabber.setPower(1);
        }

        else{
            grabber.setPower(0);
        }

        if (gamepad2.dpad_right){
            wrist.setPosition(0.39);
        }

        if (gamepad2.dpad_left) {
            wrist.setPosition(0.74);
        }

        if(gamepad2.dpad_up){
            wanVal = outVal;
        }

        if(gamepad2.dpad_down){
            wanVal = inVal;
        }

        if(0 < ExtendArm.getCurrentPosition()){
            if (gamepad2.dpad_down){
                //ExtendArm.setPower(0.01);
            }
        }
        if (10 > ExtendArm.getCurrentPosition()){
            if (gamepad2.dpad_up){
                //ExtendArm.setPower(-0.1);
            }
        }
        else {
            ExtendArm.setPower(0);
        }

        double extend = wanVal - lenVal;

        //Arm in/out stuff
        double p = 0.01;
        double d = 0.01;

        double extD = extend - prevExt;
        prevExt = extend;
        double extP = extend;

        double move = (p * extP) + (d * extD);
        //ExtendArm.setPower(move);
        lenVal = ExtendArm.getCurrentPosition();
        telemetry.addData("Extend Value", lenVal);

    }
}