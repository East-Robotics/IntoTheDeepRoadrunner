package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "1+3?", group = "Autonomous")
public class RoadRunnerActionFix extends ClassActions {

    private DcMotorEx llift;
    private DcMotorEx rlift;

    public DcMotorEx lslide;

    public DcMotorEx rslide;

    private Servo claw;

    private Servo wrist;

        public void Lift(HardwareMap hardwareMap) {
            llift = hardwareMap.get(DcMotorEx.class, "LArm");
            rlift = hardwareMap.get(DcMotorEx.class, "RArm");
            llift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            llift.setDirection(DcMotorSimple.Direction.REVERSE);
            rlift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rlift.setDirection(DcMotorSimple.Direction.FORWARD);
        }


        public void Slide(HardwareMap hardwareMap) {
            lslide = hardwareMap.get(DcMotorEx.class, "LSlide");
            rslide = hardwareMap.get(DcMotorEx.class, "RSlide");
            lslide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lslide.setDirection(DcMotorSimple.Direction.FORWARD);
            rslide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rslide.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public void Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "Claw");
        }

        public void Wrist(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(Servo.class, "Wrist");
        }

    public class Drive{
        public Action turn(double angle){
            return new TodoAction();
        }
    }


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(30.5);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(30.5,0,0))
                .waitSeconds(.3)
                .lineToX(21.5)
                .strafeToLinearHeading(new Vector2d(22.5,40.5),Math.toRadians(0));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(22.5,40.5,Math.toRadians(0)))
                .waitSeconds(0.5)
                //.lineToX(10)
                //.turn(Math.toRadians(-45));
                //.lineToYLinearHeading(45,Math.toRadians(-45));
                .strafeToLinearHeading(new Vector2d(21.5,38.5), Math.toRadians(-45));
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(21.5,38.5,Math.toRadians(-45)))
                .waitSeconds(0.5)
                //.lineToXLinearHeading(20, Math.toRadians(0));
                .strafeToLinearHeading(new Vector2d(21.5,50), Math.toRadians(0));
        //.strafeToLinearHeading(new Vector2d(11,45),Math  .toRadians(0));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(21.5,50,Math.toRadians(0)))
                .waitSeconds(0.5)
                //.lineToX(15)
                //.lineToYLinearHeading(45,Math.toRadians(-45));
                .strafeToLinearHeading(new Vector2d(21,37.5), Math.toRadians(-45));
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(21,37.5,Math.toRadians(-45)))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(19.5,54), Math.toRadians(22));
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(19.5,54,Math.toRadians(22)))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(24,37.5), Math.toRadians(-45));
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(24,37.5,Math.toRadians(-45)))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(85,1.5), Math.toRadians(-90));
        Action trajectoryActionClose1 = tab1.endTrajectory().fresh()
                //.strafeTo(new Vector2d(48, 12))
                .build();
        Action trajectoryActionClose2 = tab2.endTrajectory().fresh()
                .build();
        Action trajectoryActionClose3 = tab3.endTrajectory().fresh()
                .waitSeconds(.2)
                .build();
        Action trajectoryActionClose4 = tab4.endTrajectory().fresh()
                .waitSeconds(.2)
                .build();
        Action trajectoryActionClose5 = tab5.endTrajectory().fresh()
                .waitSeconds(.2)
                .build();
        Action trajectoryActionClose6 = tab5.endTrajectory().fresh()
                .waitSeconds(.2)
                .build();

        Action trajectoryActionClose7 = tab7.endTrajectory().fresh()
                .waitSeconds(.2)
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(lift.liftInit());

        //  Actions.runBlocking(wrist.wristBack());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();

        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();


        waitForStart();

        if (isStopRequested()) return;



        Actions.runBlocking(
                new SequentialAction(
                        //specimen
                        wrist.wristBack(),
                        tab1.build(),
                        trajectoryActionClose1,
                        wrist.wristDown(),
                        lift.liftMid(),
                        claw.openClaw(),
                        new ParallelAction(
                                wrist.wristBack(),
                                //1st sample
                                tab2.build()),
                        wrist.wristDown(),
                        lift.liftDown(),
                        claw.closeClaw(),
                        trajectoryActionClose2,
                        lift.liftUp(),
                        new ParallelAction(
                                tab3.build(),
                                slide.slideUp()
                        ),
                        wrist.wristBack(),
                        trajectoryActionClose3,
                        claw.openClaw(),
                        //2nd sample
                        new ParallelAction(
                                wrist.wristDown(),
                                slide.slideDown(),
                                tab4.build(),
                                lift.liftDown()
                        ),
                        claw.closeClaw(),
                        lift.liftUp(),
                        trajectoryActionClose4,
                        new ParallelAction(
                                tab5.build(),
                                slide.slideUp()

                        ),
                        wrist.wristBack(),
                        trajectoryActionClose5,
                        claw.openClaw(),
                        //3rd sample
                        new ParallelAction(
                                wrist.wristDown(),
                                slide.slideDown(),
                                lift.liftDown(),
                                tab6.build()
                        ),
                        claw.closeClaw(),
                        trajectoryActionClose6,
                        lift.liftUp(),
                        new ParallelAction(
                                tab7.build(),
                                slide.slideUp()
                        ),
                        wrist.wristBack(),
                        trajectoryActionClose7,
                        claw.openClaw(),
                        wrist.wristDown(),
                        //park
                        new ParallelAction(
                                slide.slideDown(),
                                lift.liftPark(),
                                tab8.build()
                        ),
                        wrist.wristBack(),
                        wrist.wristUp()

                )
        );
    }
}