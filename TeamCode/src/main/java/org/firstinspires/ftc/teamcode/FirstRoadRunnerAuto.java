package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryActionFactory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.teleops.CenterstageTestCode;

@Autonomous(name="First RoadRunner Auto")
public class FirstRoadRunnerAuto extends LinearOpMode {

public DcMotor RArm;
public DcMotor LArm;
public DcMotor LSlide;
public DcMotor RSlide;




    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Servo Claw = hardwareMap.servo.get("Claw");
        Servo Wrist = hardwareMap.servo.get("Wrist");
        RArm = hardwareMap.dcMotor.get("RArm" );
        LArm = hardwareMap.dcMotor.get("LArm");
        LSlide = hardwareMap.dcMotor.get("LSlide");
        RSlide = hardwareMap.dcMotor.get("RSlide");

        class Drive {
            public Action followTrajectory(Trajectory t) {
                return new TodoAction();
            }

            public Action turn(double angle) {
                return new TodoAction();
            }

            public Action moveToPoint(double x, double y) {
                return new TodoAction();
            }
        }

        class Shooter {
            public Action spinUp() {
                return new TodoAction();
            }

            public Action fireBall() {
                return new TodoAction();
            }

            public Action loadBall() {
                return new TodoAction();
            }
        }


        waitForStart();
        LArm.setDirection(DcMotorSimple.Direction.REVERSE);
        RArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       /* Actions.runBlocking(new SequentialAction(
                drive.turn(Math.PI / 2),
                new ParallelAction(
                        drive.followTrajectory(shootingTraj),
                        new SequentialAction(
                                shooter.spinUp(),
                                shooter.fireBall()
                        )
                )
        ));
        /*Actions.runBlocking(new ParallelAction(
                     drive.actionBuilder(new Pose2d(0, 0, 0))
                        /*.stopAndAdd(new ServoAction(Claw, .5))
                        .stopAndAdd(new ServoAction(Wrist,.5))
                        .lineToX(23)
                        .stopAndAdd(new ServoAction(Wrist,0.09))
                        .waitSeconds(1)
                        .stopAndAdd(new ServoAction(Claw,0.1))
                        .strafeTo(new Vector2d(15,32))
                        .waitSeconds(.1)
                        .lineToX(23)
                        .stopAndAdd(new ServoAction(Claw,.5))
                        .lineToX(17)
                        .turn(Math.toRadians(-45))
                        .lineToX(8)
                        .stopAndAdd(new ServoAction(Wrist,1))
                        .waitSeconds(3)
                        .stopAndAdd(new ArmUp(RArm, 0.6))
                        .stopAndAdd(new ArmUp(LArm, .6))
                        //.stopAndAdd(new SlideUP(LSlide,0.4))
                        /*.stopAndAdd(new ServoAction(Claw,0.1))
                        .waitSeconds(1)
                        .turn(Math.toRadians(45))
                        .stopAndAdd(new ServoAction(Wrist, .09))
                        .waitSeconds(1)
                        .stopAndAdd(new ServoAction(Claw,.1))
                        .waitSeconds(1)
                        .turn(Math.toRadians(-45))
                        .stopAndAdd(new ServoAction(Claw,0.5))
                        .waitSeconds(1)
                        .stopAndAdd(new ServoAction(Wrist,1))
                        .waitSeconds(1)
                        .stopAndAdd(new ServoAction(Claw,.5))
                        .waitSeconds(1)
                        .stopAndAdd(new ServoAction(Wrist,.09))
                        .waitSeconds(1)
                        .stopAndAdd(new ServoAction(Claw,0.1))
                        .waitSeconds(1)
                        .stopAndAdd(new ServoAction(Wrist, 1))
                        .build()));*/



    }


   /* public class ArmUp implements Action {

        DcMotor motor;
        double power;

        public ArmUp(DcMotor m, double p) {
            this.motor = m;
            this.power = p;
        }

            public boolean run (@NonNull TelemetryPacket telemetryPacket){
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setTargetPosition(1300);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(power);
                while (motor.isBusy()) {

                }
                motor.setPower(0.04);
                return false;
            }

    }

    public class ArmDown implements Action {

        DcMotor motor;
        double power;

        public ArmDown(DcMotor m, double p) {
            this.motor = m;
            this.power = p;
        }

        public boolean run (@NonNull TelemetryPacket telemetryPacket){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(-1300);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
            while (motor.isBusy()) {

            }
            motor.setPower(0.04);
            return false;
        }

    }

    public class SlideUP implements Action {

        DcMotor motor;
        double powered;

        public SlideUP(DcMotor m, double p) {
            this.motor = m;
            this.powered = p;
        }

        public boolean run (@NonNull TelemetryPacket telemetryPacket){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(-3000);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(powered);
            /*while (motor.isBusy()) {

            }
            motor.setPower(0.04);
            return false;
        }

    }

    public class slidedown implements Action{

        DcMotor motor;
        double powered;

        public slidedown(DcMotor m, double p) {
            this.motor = m;
            this.powered = p;
        }

        public boolean run (@NonNull TelemetryPacket telemetryPacket){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(3000);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(powered);
            while (motor.isBusy()) {

            }
            motor.setPower(0.04);
            return false;
        }

    }

    public class ServoAction implements Action {
        Servo servo;
        double position;

        public ServoAction(Servo s, double p) {
            this.servo = s;
            this.position = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo.setPosition(position);
            return false;

        }

    }*/




}
/*drive.actionBuilder(new Pose2d(0, 0, 0))
                        /*.stopAndAdd(new ServoAction(Claw, .5))
                        .stopAndAdd(new ServoAction(Wrist,.5))
                        .lineToX(23)
                        .stopAndAdd(new ServoAction(Wrist,0.09))
                        .waitSeconds(1)
                        .stopAndAdd(new ServoAction(Claw,0.1))
                        .strafeTo(new Vector2d(15,32))
                        .waitSeconds(.1)
                        .lineToX(23)
                        .stopAndAdd(new ServoAction(Claw,.5))
                        .lineToX(17)
                        .turn(Math.toRadians(-45))
                        .lineToX(8)
                        .stopAndAdd(new ServoAction(Wrist,1))
                        .waitSeconds(3)
                        //.stopAndAdd(new ArmUp(RArm, 0.6))
                        //.stopAndAdd(new ArmDown(RArm, .6))
                        .stopAndAdd(new SlideUP(RArm,0.4))
                        /*.stopAndAdd(new ArmUp(LArm,0.6))
                        .stopAndAdd(new ServoAction(Claw,0.1))
                        .waitSeconds(1)
                        .turn(Math.toRadians(45))
                        .stopAndAdd(new ServoAction(Wrist, .09))
                        .waitSeconds(1)
                        .stopAndAdd(new ServoAction(Claw,.1))
                        .waitSeconds(1)
                        .turn(Math.toRadians(-45))
                        .stopAndAdd(new ServoAction(Claw,0.5))
                        .waitSeconds(1)
                        .stopAndAdd(new ServoAction(Wrist,1))
                        .waitSeconds(1)
                        .stopAndAdd(new ServoAction(Claw,.5))
                        .waitSeconds(1)
                        .stopAndAdd(new ServoAction(Wrist,.09))
                        .waitSeconds(1)
                        .stopAndAdd(new ServoAction(Claw,0.1))
                        .waitSeconds(1)
                        .stopAndAdd(new ServoAction(Wrist, 1))
                        .build());*/
