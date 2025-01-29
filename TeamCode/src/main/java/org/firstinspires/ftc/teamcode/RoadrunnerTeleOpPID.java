/*package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


//@Config
@TeleOp(name = "RoadRunnerTeleOpPID", group = "TeleOp")
public class RoadrunnerTeleOpPID extends LinearOpMode {

    private MecanumDrive drive;

    private Pose2d targetPose = new Pose2d(30, 30, Math.toRadians(45));
    private Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));

    public void colorTelemetry() {
        telemetry.addData("redValue","%.2f", redValue);
        telemetry.addData("greenValue","%.2f", greenValue);
        telemetry.addData("blueValue","%.2f", blueValue);
        telemetry.addData("alphaValue","%.2f", alphaValue);
        telemetry.update();

    }

    public class Lift {
        private DcMotorEx llift;
        private DcMotorEx rlift;
        ;
        public Lift(HardwareMap hardwareMap) {
            llift = hardwareMap.get(DcMotorEx.class, "LArm");
            rlift = hardwareMap.get(DcMotorEx.class, "RArm");
            llift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            llift.setDirection(DcMotorSimple.Direction.REVERSE);
            rlift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rlift.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rlift.setPower(0.75);
                    llift.setPower(0.75);
                    initialized = true;
                }

                double pos = llift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2900) {
                    return true;
                } else {
                    llift.setPower(0);
                    rlift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new RoadrunnerTeleOpPID.Lift.LiftUp();
        }

        public class LiftInit implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rlift.setPower(0.8);
                    llift.setPower(0.8);
                    initialized = true;
                }

                double pos = llift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1450) {
                    return true;
                } else {
                    llift.setPower(0);
                    rlift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftInit() {
            return new RoadrunnerTeleOpPID.Lift.LiftInit();
        }

        public class LiftMid implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    llift.setPower(-0.8);
                    rlift.setPower(-0.8);
                    initialized = true;
                }

                double pos = llift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1000) {
                    return true;
                } else {
                    sleep(300);
                    llift.setPower(0);
                    rlift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftMid(){
            return new RoadrunnerTeleOpPID.Lift.LiftMid();
        }

        public class LiftPark implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    llift.setPower(-0.8);
                    rlift.setPower(-0.8);
                    initialized = true;
                }

                double pos = llift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 1800) {
                    return true;
                } else {
                    //   sleep(300);
                    llift.setPower(0);
                    rlift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftPark(){
            return new RoadrunnerTeleOpPID.Lift.LiftPark();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    llift.setPower(-0.75);
                    rlift.setPower(-0.75);
                    initialized = true;
                }

                double pos = llift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 10) {
                    return true;
                } else {
                    llift.setPower(0);
                    rlift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new RoadrunnerTeleOpPID.Lift.LiftDown();
        }
    }

    public class Slide {
        private DcMotorEx lslide;
        private DcMotorEx rslide;

        public Slide(HardwareMap hardwareMap) {
            lslide = hardwareMap.get(DcMotorEx.class, "LSlide");
            rslide = hardwareMap.get(DcMotorEx.class, "RSlide");
            lslide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lslide.setDirection(DcMotorSimple.Direction.FORWARD);
            rslide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rslide.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class Up implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rslide.setPower(-0.8);
                    lslide.setPower(-0.8);
                    initialized = true;
                }

                double pos = lslide.getCurrentPosition();
                packet.put("SlidePos", pos);
                if (pos < 2500) {
                    return true;
                } else {
                    lslide.setPower(0);
                    rslide.setPower(0);
                    return false;
                }
            }
        }
        public Action slideUp() {
            return new RoadrunnerTeleOpPID.Slide.Up();
        }

        public class Down implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lslide.setPower(0.8);
                    rslide.setPower(0.8);
                    //lslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //rslide.setMode(DcMotor.RunMode. STOP_AND_RESET_ENCODER);
                    initialized = true;
                }

                double pos = lslide.getCurrentPosition();
                packet.put("SlidePos", pos);
                if (pos > 165) {
                    return true;
                } else {
                    lslide.setPower(0);
                    rslide.setPower(0);
                    return false;
                }
            }
        }
        public Action slideDown(){
            return new RoadrunnerTeleOpPID.Slide.Down();
        }

        public class Out implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lslide.setPower(-0.8);
                    rslide.setPower(-0.8);
                    //lslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //rslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    initialized = true;
                }

                double pos = lslide.getCurrentPosition();
                packet.put("SlidePos", pos);
                if (pos < 1000) {
                    return true;
                } else {
                    lslide.setPower(0);
                    rslide.setPower(0);
                    return false;
                }
            }
        }
        public Action Slideout(){
            return new RoadrunnerTeleOpPID.Slide.Out();
        }

        public class In implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lslide.setPower(0.8);
                    rslide.setPower(0.8);
                    //lslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //rslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    initialized = true;
                }

                double pos = lslide.getCurrentPosition();
                packet.put("SlidePos", pos);
                if (pos > 215) {
                    return true;
                } else {
                    lslide.setPower(0);
                    rslide.setPower(0);
                    return false;
                }
            }
        }
        public Action Slidein(){
            return new RoadrunnerTeleOpPID.Slide.In();
        }
    }



    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "Claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.27);
                sleep(500);
                return false;
            }
        }
        public Action closeClaw() {
            return new RoadrunnerTeleOpPID.Claw.CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.01);
                sleep(500);
                return false;
            }
        }
        public Action openClaw() {
            return new RoadrunnerTeleOpPID.Claw.OpenClaw();}
    }
    public class Wrist {
        private Servo wrist;

        public Wrist(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(Servo.class, "Wrist");
        }

        public class DownWrist implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(0.3);
                sleep(500);
                return false;
            }
        }

        public Action wristDown() {
            return new RoadrunnerTeleOpPID.Wrist.DownWrist();
        }

        public class BackWrist implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wrist.setPosition(1);
                sleep(200);
                return false;
            }
        }

        public  Action wristBack() {
            return new RoadrunnerTeleOpPID.Wrist.BackWrist();
        }

        public class UpWrist implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(0.6);
                return false;
            }
        }
        public Action wristUp() {
            return new RoadrunnerTeleOpPID.Wrist.UpWrist();
        }
    }
    public class Drive{
        public Action turn(double angle){
            return new TodoAction();
        }
    }
    private DcMotor LFMotor;
    private DcMotor RFMotor;
    private DcMotor LBMotor;
    private DcMotor RBMotor;

    private DcMotor LSlide;
    private DcMotor RSlide;
    private DcMotor LArm;
    private DcMotor RArm;
    private Servo Wrist;
    private Servo Claw;


    boolean WristIsOpen = true;
    boolean ClawIsOpen = true;

    boolean lastYState = false;
    boolean currentYState = false;

    boolean lastXState = false;
    boolean currentXState = false;

    private ColorSensor colorSensor;
    private double redValue;
    private double greenValue;
    private double blueValue;
    private double alphaValue; // Light Intensity
    private double redTarget = 200;
    private double greenTarget = 200;
    private double blueTarget = 200;

    public RoadrunnerTeleOp() {
        robot = new Robot(hardwareMap);
    }

    public void initHardware( ){
        initColorSensor();
    }
    public void initColorSensor() {
        colorSensor = hardwareMap.get(ColorSensor.class, "ClawColor");
    }
    public void getColor() {
        redValue = colorSensor.red();
        greenValue = colorSensor.green();
        blueValue = colorSensor.blue();
        alphaValue = colorSensor.alpha();
    }




    @Override
    public void runOpMode() throws InterruptedException {
        //drive = new MecanumDrive(hardwareMap);
       // drive.setPoseEstimate(new Pose2d(0,0,0));
        drive.updatePoseEstimate();




        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(20,20), Math.toRadians(45));




        //Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        //MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

       Claw claw = new Claw(hardwareMap);
       Lift lift = new Lift(hardwareMap);
       Slide slide = new Slide(hardwareMap);
       Wrist wrist = new Wrist(hardwareMap);

        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LSlide = hardwareMap.get(DcMotor.class, "LSlide");
        RSlide = hardwareMap.get(DcMotor.class, "RSlide");
        LArm = hardwareMap.get(DcMotor.class, "LArm");
        RArm = hardwareMap.get(DcMotor.class, "RArm");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Claw = hardwareMap.get(Servo.class, "Claw");

       // ClassActions.Claw claw = (ClassActions.Claw) hardwareMap.get(Servo.class, "Claw");
        //ClassActions.Wrist wrist = (ClassActions.Wrist) hardwareMap.get(Servo.class, "Wrist");
       // ClassActions.Slide slide = (ClassActions.Slide) hardwareMap.get(DcMotor.class, "LSlide");







        //Encoders
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //localizer = new TwoDeadWheelLocalizer(hardwareMap, lazyImu.get(), PARAMS.inPerTick);

        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        LFMotor.setDirection(DcMotor.Direction.REVERSE);


        //   LArm.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Running");
        telemetry.update();

        LArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LArm.setPower(0.04);
        RArm.setPower(-0.04);



        //Wrist.setDirection(Servo.Direction.REVERSE);
        //claw.setDirection(Servo.Direction.REVERSE);

         LBMotor.setDirection(DcMotor.Direction.REVERSE);
        //LArm.setDirection(DcMotor.Direction.REVERSE);



        currentYState = gamepad2.y;
        currentXState = gamepad2.x;

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        waitForStart();

        imu.resetYaw();

        // vision here that outputs position
        int visionOutputPosition = 1;



        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

            initHardware();
            while (!isStarted()) {
                getColor();
             //   colorTelemetry();
            }
            while (opModeIsActive()) {
                getColor();
                //colorTelemetry();

                int  LArmPos = LArm.getCurrentPosition();
                int  RArmPos = RArm.getCurrentPosition();
                int  LSlidePos = LSlide.getCurrentPosition();
                int  RSlidePos = RSlide.getCurrentPosition();
                double WristPos = Wrist.getPosition();

                TwoDeadWheelLocalizer.Params Position = new TwoDeadWheelLocalizer.Params();

                PositionVelocityPair parPosVel = par.getPositionAndVelocity();

                lastParPos = parPosVel.position;
                lastPerpPos = perpPosVel.position;
                lastHeading = heading;


                telemetry.addData("RArmPos", RArmPos);
                telemetry.addData("LArmPos", LArmPos);
                telemetry.addData("LSlidePos", LSlidePos);
                telemetry.addData("RSlidePos", RSlidePos);
                telemetry.addData("ClawPos", ClawIsOpen);
                telemetry.addData("Wrist", WristIsOpen);
                telemetry.update();
                currentXState = gamepad2.x;
                telemetry.addData("x", initialPose.position);
                telemetry.addData("Y", initialPose.position);
                telemetry.addData("heading", initialPose.heading);

            double py = -gamepad1.left_stick_y;
            double px = gamepad1.left_stick_x;
            double pa = gamepad1.right_stick_x;

            double Kp = 0.1;
            double Ki = 0.1;
            double Kd = 0.1;

            double reference = 0.2;

            double integralSum = 0;

            double lastError = 0;
            ElapsedTime timer = new ElapsedTime();

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            botHeading -= Math.toRadians(90);

            if (gamepad1.start) {
                imu.resetYaw();
            }

            // Rotate the movement direction counter to the bot's rotation
            double rotX = px * Math.cos(-botHeading) - py * Math.sin(-botHeading);
            double rotY = px * Math.sin(-botHeading) + py * Math.cos(-botHeading);

            rotX = rotX * 1.5;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(pa), 1);

            double LFtgtPower = (rotY + rotX + pa)/denominator;
            double LBtgtPower = (-rotY + rotX - pa) /denominator;
            double RFtgtPower = (rotY - rotX - pa)/denominator;
            double RBtgtPower = (rotY + rotX - pa)/denominator;

            RFMotor.setPower(RFtgtPower);

            LFMotor.setPower(LFtgtPower);

            RBMotor.setPower(RBtgtPower);

            LBMotor.setPower(LBtgtPower);

            while (setPointIsNotReached) {


                    // obtain the encoder position
                double encoderPosition = LFMotor.getCurrentPosition();
                    // calculate the error
                double error = reference - encoderPosition;

                    // rate of change of the error
                double derivative = (error - lastError) / timer.seconds();

                    // sum of all error over time
                integralSum = integralSum + (error * timer.seconds());

                double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

                LFMotor.setPower(out);

                lastError = error;

                    // reset the timer for next time
                timer.reset();

                }

                if (WristIsOpen && gamepad2.y) {
                 Actions.runBlocking(
                            new ParallelAction(
                                    wrist.wristBack()
                            )
                    );
                WristIsOpen = false;
            }
            if (!WristIsOpen && gamepad2.y) {
                  Actions.runBlocking(
                        new ParallelAction(
                                wrist.wristDown()
                         )
                  );
                  WristIsOpen = true;
            }

            if (gamepad1.b){
                LArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                LSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                LArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }

                if (currentXState && !lastXState) {
                    ClawIsOpen = !ClawIsOpen;
                }

                lastXState = currentXState;

                if (ClawIsOpen) {
                    Claw.setPosition(0.01);

                } else {
                    Claw.setPosition(0.27);
                }


                //Claw toggle
           if (currentXState && !lastXState) {
                ClawIsOpen = !ClawIsOpen;
            }

            lastXState = currentXState;
*/

               /* if (ClawIsOpen && gamepad2.x){
                Actions.runBlocking(
                        new ParallelAction(
                                claw.openClaw()
                        )

                );
                ClawIsOpen = false;
                    telemetry.addData("ClawPos", ClawIsOpen);
                    telemetry.update();

            }
            if(!ClawIsOpen && gamepad2.x) {
                Actions.runBlocking(
                        new ParallelAction(
                               claw.closeClaw()

                        )
                );
                ClawIsOpen = true;
                telemetry.addData("ClawPos", ClawIsOpen);
                telemetry.update();
            }

//Arm control
            if (gamepad2.right_bumper) {//down
                if ((RArmPos <200)) {
                    LArm.setPower(0);
                    RArm.setPower(0);
                    LSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    RSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    LSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                else {
                    LArm.setPower(-0.85);
                    RArm.setPower(-0.85);
                }

            }
            else if(gamepad2.left_bumper){//up
                if ((RArmPos> 2900)) {
                    LArm.setPower(0);
                    RArm.setPower(0);
                }
                else {
                    LArm.setPower(0.85);
                    RArm.setPower(0.85);
                }
            }
            else{
                LArm.setPower(-0.08);
                RArm.setPower(0.08);
            }
//Slide control
            if (gamepad1.right_bumper){//down
                LSlide.setPower(0.9);
                RSlide.setPower(0.9);
            }
            else if (gamepad1.left_bumper){//up
                if ((LArmPos < 1300) && (RArmPos < 1300) && ((LSlidePos > 900))){
                    LSlide.setPower(0);
                    RSlide.setPower(0);
                }
                else{
                    LSlide.setPower(-0.9);
                    RSlide.setPower(-0.9);
                }
            }
            else{
                LSlide.setPower(-0.04);
                RSlide.setPower(-0.04);
            }

            if (redValue> redTarget) {
                claw.closeClaw();
            }

            if (greenValue> greenTarget) {
                claw.closeClaw();
            }

            if (blueValue> blueTarget) {
                claw.closeClaw();
            }


                if (gamepad1.dpad_up) {
                Actions.runBlocking(
                        new ParallelAction(
                        new SequentialAction(
                        //        lift.liftUp(),
                                slide.slideUp()
                        )
                        )
                );
            }

            if (gamepad1.dpad_down) {
                Actions.runBlocking(
                        new ParallelAction(
                              //  claw.closeClaw(),
                                slide.slideDown()
                              //  lift.liftDown()
                        )
                );
            }
                if (gamepad1.x) {
                    // Trigger movement to the target position when the A button is pressed
                    Actions.runBlocking(
                            new SequentialAction(
                                    tab1.build()

                            )
                    );
                }

        }
        if (isStopRequested()) return;




    }
}
*/