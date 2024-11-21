package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp")
public class teleop_v2 extends OpMode {
    //name motors
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor ArmMotor;
    DcMotor ExtendArm;
    CRServo grabber;
    Servo wrist;
    //name servos


    double desVal = 0;
    double wanVal = 0;

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
    public void start() {
        wrist.setPosition(0.39);
    }


    //loop start
    @Override
    public void loop () {
        Drive();
        Arm();
        Extend();
//        NewExtend();
//        Grabber();

        //If B button held, make grabber spit out


        if (gamepad2.dpad_right){
            wrist.setPosition(0.39);
        }

        if (gamepad2.dpad_left) {
            wrist.setPosition(0.74);
        }






    }

    public void Drive(){
        float y = gamepad1.left_stick_y/2;
        float x = gamepad1.left_stick_x/2;
        float t = gamepad1.right_stick_x/2;

        motorFL.setPower(y - x - t);
        motorFR.setPower(-y - x - t);
        motorBL.setPower(y + x - t);
        motorBR.setPower(-y + x - t);
    }
    public void Arm(){
        float ea = -gamepad2.right_stick_y/2;
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (ArmMotor.getCurrentPosition() >= -10){
            if (ArmMotor.getCurrentPosition() < 3000) {
                ArmMotor.setPower(ea);
            }
        }
        double mvVal = ArmMotor.getCurrentPosition();
        double eror = dsVal - mvVal;

        if (ea != 0){
            dsVal = ArmMotor.getCurrentPosition();
        }
        if (gamepad2.a){
            ArmMotor.setPower(0);
            dsVal = ArmMotor.getCurrentPosition();
        }
        if (!gamepad2.a) {
            if (ea == 0) {
                double op = 0.0005;
                double od = 0.0005;

                double errod = eror - prevEr;
                prevEr = eror;
                double errop = eror;

                double Go = (op * errop) + (od * errod);
                ArmMotor.setPower(Go);
                mvVal = ArmMotor.getCurrentPosition();
                telemetry.addData("Arm Value", ArmMotor.getCurrentPosition());
            }
        }
    }
    public void Extend(){
        double outVal = -1310;
        double inVal = 0;
        double lenVal = ExtendArm.getCurrentPosition();

        if(gamepad2.dpad_up){
            wanVal = outVal;
        }

        if(gamepad2.dpad_down){
            wanVal = inVal;
        }
        double extend = wanVal - lenVal;

        double p = 0.01;
        double d = 0.01;

        double extD = extend - prevExt;
        prevExt = extend;
        double extP = extend;

        double move = (p * extP) + (d * extD);
        ExtendArm.setPower(move);
        lenVal = ExtendArm.getCurrentPosition();
        telemetry.addData("Extend Value", lenVal);
    }

    public void NewExtend(){
        if(0 < ExtendArm.getCurrentPosition()){
            if (gamepad2.dpad_down){
                //ExtendArm.setPower(0.01);
            }
        }
        if (10 > ExtendArm.getCurrentPosition()){
            if (gamepad2.dpad_up){
                //ExtendArm.setPower(-0.01);
            }
        }
        else {
            ExtendArm.setPower(0);
        }
    }

    public void Grabber() {
        if(gamepad2.x) {
            grabber.setPower(-1);
        }
        if(gamepad2.b){
            grabber.setPower(1);
        }
        else{
            grabber.setPower(0);
        }
    }
}