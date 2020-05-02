import java.util.ArrayList;

final int WIDTH = 640;
final int HEIGHT = 360;

final float dt = 1.0 / 60;

Robot robot = new Robot(new PVector(50, 50));
MecanumController controller = new MecanumController(robot);

void settings(){
    size(WIDTH, HEIGHT);
}

void setup() {
    rectMode(CENTER);
    controller.strafeToPose(new PVector(300, 300), -PI / 4);
    //robot.moveVector(new PVector(30, 0), 0.0);
}

void draw() {
    background(0);
    controller.update(dt);
    robot.update(dt);
    robot.render();
    controller.render();
}
