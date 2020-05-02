public class Robot {
    
    public static final float ROBOT_SIZE = 30.0;
    public final float HEIGHT_FACTOR = (float) Math.sqrt(3) / 2;
 
    private PVector position = new PVector(0.0, 0.0);
    private float heading = 0.0;
    
    private PVector velocity = new PVector(0.0, 0.0);
    private float angularVelocity = 0.0;

    public Robot(PVector position) {
        this.position = position;
    }
    
    public PVector getPosition() {
        return position;   
    }
    
    public float getHeading() {
        return heading;
    }
    
    public void moveVector(PVector vel, float omega) {
        velocity.set(vel.x, vel.y);
        angularVelocity = omega;
    }
    
    public void stop() {
        moveVector(new PVector(0.0, 0.0), 0.0f);
    }
    
    public void update(float dt) {
        position.add(velocity.copy().rotate(heading).mult(dt));
        heading += angularVelocity * dt;
    }
    
    public void render() {
        fill(255, 255, 255);
        strokeWeight(0);
        pushMatrix();
        translate(position.x, position.y);
        rotate(-heading);
        triangle(HEIGHT_FACTOR * ROBOT_SIZE, 0, -HEIGHT_FACTOR * ROBOT_SIZE / 2, ROBOT_SIZE / 2, -HEIGHT_FACTOR * ROBOT_SIZE / 2, -ROBOT_SIZE / 2);
        popMatrix();
    }
}
