public class MecanumController {
 
    private float MAX_VEL = 40;
    private float MAX_OMEGA = PI / 12;
    
    private Robot robot;
    
    private boolean isStrafing = false;
    private PVector targetPos;
    private float targetHeading;
    
    private PVector dirVector;
    
    private PIDController distancePid = new PIDController(
        new PIDController.Gains(0.03, 0.0, 0.002));

    private PIDController anglePid = new PIDController(
        new PIDController.Gains(2.0, 0.0, 0.0));
    
    public MecanumController(Robot robot) {
        this.robot = robot;
    }
    
    public void strafeToPose(PVector pos, float heading) {
        targetPos = pos;
        targetHeading = heading;
        
        distancePid.reset();
        distancePid.setSetpoint(0.0);
        anglePid.reset();
        anglePid.setSetpoint(heading);
        
        isStrafing = true;
    }
    
    public void update(float dt) {
        if (isStrafing) {
            PVector currentPos = robot.getPosition();
            dirVector = PVector.sub(targetPos, currentPos);
            
            float distanceOutput = distancePid.calculate(dirVector.mag(), dt);
            dirVector = dirVector.normalize().mult(-distanceOutput * MAX_VEL);
            float angleOutput = MAX_OMEGA * anglePid.calculate(robot.getHeading(), dt);
            
            System.out.println("Distance PID: " + distanceOutput + " | Angle PID: " + angleOutput);
            
            robot.moveVector(dirVector.copy().rotate(-robot.getHeading()), angleOutput);
            
            if (distancePid.atSetpoint() && anglePid.atSetpoint()) {
                isStrafing = false;
                robot.stop();
                System.out.println("Done!");
                
                strafeToPose(new PVector(500, 300), PI / 4);
            }
        }
    }
    
    public void render() {
        if (isStrafing) {
            strokeWeight(0);
            fill(255, 0, 0);
            circle(targetPos.x, targetPos.y, 10);
            
            PVector pos = robot.getPosition();
            PVector end = PVector.add(pos, dirVector);
            System.out.println(end);
            stroke(0, 255, 0);
            strokeWeight(4);
            line(pos.x, pos.y, end.x, end.y);
        }
    }
}
