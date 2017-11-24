package org.myrobotlab.service;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.myrobotlab.framework.Service;
import org.myrobotlab.framework.ServiceType;
import org.myrobotlab.jme3.InMoov3DApp;
import org.myrobotlab.kinematics.DHLink;
import org.myrobotlab.kinematics.DHRobotArm;
import org.myrobotlab.kinematics.Matrix;
import org.myrobotlab.kinematics.Point;
import org.myrobotlab.logging.Level;
import org.myrobotlab.logging.LoggerFactory;
import org.myrobotlab.logging.LoggingFactory;
import org.myrobotlab.math.MathUtils;
import org.myrobotlab.service.Servo.IKData;
import org.myrobotlab.service.data.JoystickData;
import org.myrobotlab.service.interfaces.IKJointAnglePublisher;
import org.myrobotlab.service.interfaces.PointsListener;
import org.slf4j.Logger;

/**
 * 
 * InverseKinematics3D - This class provides a 3D based inverse kinematics
 * implementation that allows you to specify the robot arm geometry based on DH
 * Parameters. This will use a pseudo-inverse jacobian gradient descent approach
 * to move the end effector to the desired x,y,z positions in space with respect
 * to the base frame.
 * 
 * Rotation and Orientation information is not currently supported. (but should
 * be easy to add)
 *
 * @author kwatters
 * 
 */
public class InverseKinematics3D extends Service implements IKJointAnglePublisher, PointsListener {

  private static final long serialVersionUID = 1L;
  public final static Logger log = LoggerFactory.getLogger(InverseKinematics3D.class.getCanonicalName());

  private DHRobotArm currentArm = null;

  // we will track the joystick input to specify our velocity.
  private Point joystickLinearVelocity = new Point(0, 0, 0, 0, 0, 0);

  private Matrix inputMatrix = null;
  private Matrix baseToWorldTransform = null;
  private Point virtualArmOrigin = null;
  private Point worldToVirtualTranslation = new Point(0, 0, 0, 0, 0, 0);

  transient private InMoov3DApp vinMoovApp = null;

  transient InputTrackingThread trackingThread = null;
  private double[] targetPoint;

  enum CoordinateFrame {
    WORLD, // World fixed frame
    WORLD_VIRTUAL,  // Virtual Inmoov world frame
    BASE_ARM, // Arm base joint frame
    // LOCAL // TODO TCP mobile frame
  }

  /**
   * A class to abstract the reference frame of the points and to convert
   * between them. The stored point is in WORLD coordinates.
   */
  class Position extends Point {
    private static final long serialVersionUID = 1L;
    private boolean isOriented = false;

    Position(double x, double y, double z, CoordinateFrame frame) {
      super(x, y, z, 0, 0, 0);
      toWorld(frame);
    }
    
    Position(double x, double y, double z, double roll, double pitch, double yaw, CoordinateFrame frame) {
      super(x, y, z, roll, pitch, yaw);
      toWorld(frame);
      isOriented = true;
    }

    // Convert the point to WORLD coordinates
    Position(Point p, CoordinateFrame frame) {
      super(p);
      toWorld(frame);
    }

    private void toWorld(CoordinateFrame inputFrame) {
      switch (inputFrame) {
      case WORLD:
        break;
      case BASE_ARM:
        if (baseToWorldTransform != null) {
          Matrix tr = new Matrix(4, 1);
          tr.elements[0][0] = getX();
          tr.elements[1][0] = getY();
          tr.elements[2][0] = getZ();
          tr.elements[3][0] = 1;
          tr = baseToWorldTransform.multiply(tr);
          setX(tr.elements[0][0]);
          setY(tr.elements[1][0]);
          setZ(tr.elements[2][0]);
        }
        break;
      case WORLD_VIRTUAL:
        Point point = subtract(worldToVirtualTranslation);
        setX(point.getX());
        setY(point.getY());
        setZ(point.getZ());
        break;
      }
    }

    public Point getPoint(CoordinateFrame frame) {
      Point ret = new Point(this);
      switch (frame) {
      case WORLD:
        break;
      case BASE_ARM:
        Matrix tr = new Matrix(4, 1);
        tr.elements[0][0] = getX();
        tr.elements[1][0] = getY();
        tr.elements[2][0] = getZ();
        tr.elements[3][0] = 1;
        tr = baseToWorldTransform.homogeneousTransformInverse().multiply(tr);
        ret.setX(tr.elements[0][0]);
        ret.setY(tr.elements[1][0]);
        ret.setZ(tr.elements[2][0]);
        break;
      case WORLD_VIRTUAL:
        Point point = add(worldToVirtualTranslation);
        ret.setX(point.getX());
        ret.setY(point.getY());
        ret.setZ(point.getZ());
        break;
      }
      return ret;
    }

    public boolean hasOrientation() {
      return isOriented;
    }
  }

  public InverseKinematics3D(String n) {
    super(n);

    // Default init
    double[][] rightArmDH = {
        {  0,   +90,   -40,  90},
        { 76,   +90,     0, -90},
        {285,   -90,    30,  90},
        {-25,   -90,  -290,   0}
    };
    double[] armZYX = {0, 0, 90};          //Roll, Pitch, Yaw of arm wtr to world frame
    double[] armOrigin = {144, 0, 370};   // With respect to world frame

    initArm("RIGHT", rightArmDH);
    // Setup transform to map arm points to world points
    setArmBaseFrame(armOrigin[0], armOrigin[1], armOrigin[2], armZYX[0], armZYX[1], armZYX[2]);
    
    centerAllJoints();
  }

  public void initArm(String side, double[][] dhParams) {
    if ("RIGHT".equals(side)) {
      InMoovArm arm = (InMoovArm) Runtime.getService("i01.rightArm");
      if (arm != null) {
        DHRobotArm dhRobotArm = arm.getDHRobotArm(this, dhParams);
        setCurrentArm(dhRobotArm);
        // dhRobotArm.setIk3D(this);
        addListener("publishJointAngles", arm.getName(), "onJointAngles");

        // Update virtualInmoov parameters
        if (vinMoovApp == null) {
          InMoov inMoov = (InMoov) Runtime.getService("i01");
          try {
            vinMoovApp = inMoov.startVinMoov();
            // Update geometry from the owning thread DO THIS with 5DOF matrix in python
            //inMoov.invoke("setArmGeometry", side, dhParams);
            float[] virtualArmOriginF = vinMoovApp.getVirtualArmOrigin();
            virtualArmOrigin = new Point(virtualArmOriginF[0], virtualArmOriginF[1], virtualArmOriginF[2], 0, 0, 0);
          } catch (InterruptedException e) {
          }
        }
      } else {
        log.error("Cannot start InverseKinematics3D service for RightArm");
      }
    } else {
      log.error("Unsupported arm \"%s\" for InverseKinematics3D", side);
    }
  }

  public void startTracking() {
    log.info(String.format("startTracking - starting new joystick input tracking thread %s_tracking", getName()));
    if (trackingThread != null) {
      stopTracking();
    }
    trackingThread = new InputTrackingThread(String.format("%s_tracking", getName()));
    trackingThread.start();
  }

  public void stopTracking() {
    if (trackingThread != null) {
      trackingThread.setTracking(false);
    }
  }

  public class InputTrackingThread extends Thread {

    private boolean isTracking = false;

    public InputTrackingThread(String name) {
      super(name);
    }

    @Override
    public void run() {

      // Ok, here we are. if we're running..
      // we should be updating the move to based on the velocities
      // that are being tracked with the joystick.

      // how many ms to wait between movements.
      long pollInterval = 250;

      isTracking = true;
      long now = System.currentTimeMillis();
      while (isTracking) {
        long pause = now + pollInterval - System.currentTimeMillis();
        try {
          // the number of milliseconds until we update the position
          Thread.sleep(pause);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          log.info("Interrupted tracking thread.");
          e.printStackTrace();
          isTracking = false;
        }
        // lets get the current position
        // current position + velocity * time
        Point current = currentPosition();
        Point targetPoint = current.add(joystickLinearVelocity.multiplyXYZ(pollInterval / 1000.0));
        if (!targetPoint.equals(current)) {
          log.info("Velocity: {} Old: {} New: {}", joystickLinearVelocity, current, targetPoint);
        }

        invoke("publishTracking", targetPoint);
        moveTo(targetPoint);
        // update current timestamp to determine how long we should wait
        // before the next moveTo is called.
        now = System.currentTimeMillis();
      }

    }

    public boolean isTracking() {
      return isTracking;
    }

    public void setTracking(boolean isTracking) {
      this.isTracking = isTracking;
    }
  }

  public Point currentPosition() {
    return currentArm.getPalmPosition();
  }

  public void moveTo(double x, double y, double z) {
    Position position = new Position(x, y, z, CoordinateFrame.WORLD);
    moveTo(position);
  }

  public void moveTo(double x, double y, double z, String frameStr) {
    Position position = new Position(x, y, z, getCoordinateFrame(frameStr));
    moveTo(position);
  }

  public void moveTo(double x, double y, double z, double roll, double pitch, double yaw, String frameStr) {
    Position position = new Position(x, y, z, roll, pitch, yaw, getCoordinateFrame(frameStr));
    moveTo(position);
  }
  
  private CoordinateFrame getCoordinateFrame(String frameStr) {
    CoordinateFrame frame;
    try {
      frame = CoordinateFrame.valueOf(frameStr);
    } catch (IllegalArgumentException e) {
      frame = CoordinateFrame.WORLD;
      log.warn("moveTo is unable to find the coordinate frame for name: %s. Defaulting to WORLD", frameStr);
    }
    return frame;
  }

  /**
   * This create a rotation and translation matrix that will be applied on the
   * "moveTo" call.
   * 
   * @param dx
   *          - x axis translation
   * @param dy
   *          - y axis translation
   * @param dz
   *          - z axis translation
   * @param roll
   *          - rotation about x (in degrees)
   * @param pitch
   *          - rotation about y (in degrees)
   * @param yaw
   *          - rotation about z (in degrees)
   * @return a matrix that represents the rotation/translation matrix
   */
  public Matrix createInputMatrix(double dx, double dy, double dz, double roll, double pitch, double yaw) {
    roll = MathUtils.degToRad(roll);
    pitch = MathUtils.degToRad(pitch);
    yaw = MathUtils.degToRad(yaw);
    Matrix trMatrix = Matrix.translation(dx, dy, dz);
    Matrix rotMatrix = Matrix.zRotation(yaw).multiply(Matrix.yRotation(pitch)).multiply(Matrix.xRotation(roll));
    inputMatrix = trMatrix.multiply(rotMatrix);
    return inputMatrix;
  }

  /**
   * Sets the fixed position of the arm base joint in world frame
   */
  public void setArmBaseFrame(double dx, double dy, double dz, double roll, double pitch, double yaw) {
    roll = MathUtils.degToRad(roll);
    pitch = MathUtils.degToRad(pitch);
    yaw = MathUtils.degToRad(yaw);
    Matrix trMatrix = Matrix.translation(dx, dy, dz);
    Matrix rotMatrix = Matrix.zRotation(roll).multiply(Matrix.yRotation(pitch)).multiply(Matrix.xRotation(yaw));
    baseToWorldTransform = trMatrix.multiply(rotMatrix);
    Point armOrigin = new Point(dx, dy, dz, 0, 0, 0);
    // TODO check this
    worldToVirtualTranslation = armOrigin.subtract(virtualArmOrigin);
  }

  /**
   * @param pIn
   *          point to be transformed
   * @param inFrame
   *          Input coordinate frame
   * @return point in the BASE_ARM frame, which is used by the inverse
   *         kinematics solver
   */
  // TODO Maybe move this and make it support any output frame
  public Point rotateAndTranslate(Point pIn) {
    Matrix pOM = new Matrix(4, 1);
    pOM.elements[0][0] = pIn.getX();
    pOM.elements[1][0] = pIn.getY();
    pOM.elements[2][0] = pIn.getZ();
    pOM.elements[3][0] = 1;

    // apply final transform
    if (inputMatrix != null) {
      pOM = inputMatrix.multiply(pOM);
    }

    // TODO: compute the roll pitch yaw
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    Point pOut = new Point(pOM.elements[0][0], pOM.elements[1][0], pOM.elements[2][0], roll, pitch, yaw);
    return pOut;
  }

  public void zeroAllJoints() {
    currentArm.zeroAllJoints();
    publishTelemetry();
  }

  public void centerAllJoints() {
    currentArm.centerAllJoints();
    publishTelemetry();
  }

  public void moveTo(Point p) {
    moveTo(new Position(p, CoordinateFrame.BASE_ARM));
  }

  public void moveTo(Position pos) {
    Point p = rotateAndTranslate(pos.getPoint(CoordinateFrame.BASE_ARM));
    boolean success = currentArm.moveToGoal(p, pos.hasOrientation());

    if (success) {
      /*if (vinMoovApp != null) {
        vinMoovApp.addPoint(pos.getPoint(CoordinateFrame.WORLD_VIRTUAL));
      }*/
      publishTelemetry();
    }
  }

  public void publishTelemetry() {
    Map<String, Double> angleMap = new HashMap<String, Double>();
    for (DHLink l : currentArm.getLinks()) {
      String jointName = l.getName();
      double theta = l.getPositionValueDeg();
      // angles between 0 - 360 degrees.. not sure what people will really want?
      // - 180 to + 180 ?
      angleMap.put(jointName, theta);
    }
    invoke("publishJointAngles", angleMap);

    // publishPositions();
  }

  private void publishPositions() {
    // we want to publish the joint positions
    // this way we can render on the web gui..
    double[][] jointPositionMap = createJointPositionMap();
    // TODO: pass a better datastructure?
    invoke("publishJointPositions", (Object) jointPositionMap);

    // Make an array with tcpPosition with index 0->BASE_ARM and 1->WORLD
    // coordinates
    double[][] tcp = new double[2][];
    tcp[0] = jointPositionMap[jointPositionMap.length - 1];
    Point wPos = new Position(tcp[0][0], tcp[0][1], tcp[0][2], CoordinateFrame.BASE_ARM)
        .getPoint(CoordinateFrame.WORLD);
    tcp[1] = new double[] { wPos.getX(), wPos.getY(), wPos.getZ() };
    invoke("publishTcpPosition", (Object) tcp);
  }

  public double[][] createJointPositionMap() {

    double[][] jointPositionMap = new double[currentArm.getNumLinks() + 1][3];

    // first position is the origin... second is the end of the first link
    jointPositionMap[0][0] = 0;
    jointPositionMap[0][1] = 0;
    jointPositionMap[0][2] = 0;

    for (int i = 1; i <= currentArm.getNumLinks(); i++) {
      Point jp = currentArm.getJointPosition(i - 1);
      jointPositionMap[i][0] = jp.getX();
      jointPositionMap[i][1] = jp.getY();
      jointPositionMap[i][2] = jp.getZ();
    }
    return jointPositionMap;
  }

  public DHRobotArm getCurrentArm() {
    return currentArm;
  }

  public void setCurrentArm(DHRobotArm currentArm) {
    this.currentArm = currentArm;
  }

  public static void main(String[] args) throws Exception {
    LoggingFactory.getInstance().configure();
    LoggingFactory.getInstance().setLevel(Level.INFO);

    Runtime.createAndStart("python", "Python");
    Runtime.createAndStart("gui", "SwingGui");

    InverseKinematics3D inversekinematics = (InverseKinematics3D) Runtime.start("ik3d", "InverseKinematics3D");
    // InverseKinematics3D inversekinematics = new InverseKinematics3D("iksvc");
    inversekinematics.setCurrentArm(InMoovArm.getDHRobotArm());
    //
    // inversekinematics.getCurrentArm().setIk3D(inversekinematics);
    // Create a new DH Arm.. simpler for initial testing.
    // d , r, theta , alpha
    // DHRobotArm testArm = new DHRobotArm();
    // testArm.addLink(new DHLink("one" ,400,0,0,90));
    // testArm.addLink(new DHLink("two" ,300,0,0,90));
    // testArm.addLink(new DHLink("three",200,0,0,0));
    // testArm.addLink(new DHLink("two", 0,0,0,0));
    // inversekinematics.setCurrentArm(testArm);
    // set up our input translation/rotation
    //
    // if (false) {
    // double dx = 400.0;
    // double dy = -600.0;
    // double dz = -350.0;
    // double roll = 0.0;
    // double pitch = 0.0;
    // double yaw = 0.0;
    // inversekinematics.createInputMatrix(dx, dy, dz, roll, pitch, yaw);
    // }

    // Rest position...
    // Point rest = new Point(100,-300,0,0,0,0);
    // rest.
    // inversekinematics.moveTo(rest);

    // LeapMotion lm = (LeapMotion)Runtime.start("leap", "LeapMotion");
    // lm.addPointsListener(inversekinematics);

    boolean attached = true;
    if (attached) {
      // set up the left inmoov arm
      InMoovArm leftArm = (InMoovArm) Runtime.start("leftArm", "InMoovArm");
      leftArm.connect("COM21");
      // leftArm.omoplate.setMinMax(0, 180);
      // attach the publish joint angles to the on JointAngles for the inmoov
      // arm.
      inversekinematics.addListener("publishJointAngles", leftArm.getName(), "onJointAngles");
    }

    // Runtime.createAndStart("gui", "SwingGui");
    // OpenCV cv1 = (OpenCV)Runtime.createAndStart("cv1", "OpenCV");
    // OpenCVFilterAffine aff1 = new OpenCVFilterAffine("aff1");
    // aff1.setAngle(270);
    // aff1.setDx(-80);
    // aff1.setDy(-80);
    // cv1.addFilter(aff1);
    //
    // cv1.setCameraIndex(0);
    // cv1.capture();
    // cv1.undockDisplay(true);

    /*
     * SwingGui gui = new SwingGui("gui"); gui.startService();
     */

    Joystick joystick = (Joystick) Runtime.start("joystick", "Joystick");
    joystick.setController(2);

    // joystick.startPolling();

    // attach the joystick input to the ik3d service.
    joystick.addInputListener(inversekinematics);

    Runtime.start("webgui", "WebGui");
    Runtime.start("log", "Log");
  }

  @Override
  public Map<String, Double> publishJointAngles(HashMap<String, Double> angleMap) {
    // TODO Auto-generated method stub
    return angleMap;
  }

  public double[][] publishJointPositions(double[][] jointPositionMap) {
    return jointPositionMap;
  }

  public double[][] publishTcpPosition(double[][] tcpPosition) {
    return tcpPosition;
  }

  public Point publishTracking(Point tracking) {
    return tracking;
  }

  @Override
  public void onPoints(List<Point> points) {
    // TODO : move input matrix translation to here? or somewhere?
    // TODO: also don't like that i'm going to just say take the first point
    // now.
    // TODO: points should probably be a map, each point should have a name ?
    moveTo(points.get(0));
  }

  public void onJoystickInput(JoystickData input) {

    // a few control button pushes
    // Ok, lets say the the "a" button starts tracking
    if ("0".equals(input.id)) {
      log.info("Start Tracking button pushed.");
      startTracking();
    } else if ("1".equals(input.id)) {
      stopTracking();
    }
    // and the "b" button stops tracking
    // TODO: use the joystick input to drive the "moveTo" command.
    // TODO: joystick listener interface?
    // input.id
    // input.value
    // depending on input we want to get the current position and move in some
    // direction.
    // or potentially stay in the same place..
    // we start at the origin
    // initially at rest.
    // we can set the velocities to be equal to the joystick inputs
    // with some gain/amplification.
    // Ok, so this will track the y,rx,ry inputs from the joystick as x,y,z
    // velocities

    // we want to have a minimum threshold o/w we set the value to zero
    // quantize
    float threshold = 0.1F;
    if (Math.abs(input.value) < threshold) {
      input.value = 0.0F;
    }

    double totalGain = 100.0;
    double xGain = totalGain;
    // invert y control.
    double yGain = -1.0 * totalGain;
    double zGain = totalGain;
    if ("x".equals(input.id)) {
      // x axis control (left/right)
      joystickLinearVelocity.setX(input.value * xGain);
    } else if ("y".equals(input.id)) {
      // y axis control (up/down)
      joystickLinearVelocity.setY(input.value * yGain);
    }
    if ("ry".equals(input.id)) {
      // z axis control (forward / backwards)
      joystickLinearVelocity.setZ(input.value * zGain);
    }
    // log.info("Linear Velocity : {}", joystickLinearVelocity);
    // on a loop I want to sample the current joystickLinearVelocity
    // at some interval and move the current position by the new dx,dy,dz
    // computed based
    // off the input from the joystick.
    // relying on the current position is probably bad.
    // TODO: track the desired position independently of the current position.
    // we will allow translation, x,y,z
    // for the input point.
  }
  
  public void onIKServoEvent(IKData data) {
    for (DHLink l : currentArm.getLinks()) {
      if (l.getName().equals(data.name)) {
        l.addPositionValue(data.pos);
        l.setState(data.state);
        l.setVelocity(data.velocity);
        l.setTargetPos(data.targetPos);
        l.setCurrentPos(data.pos);
      }
    }
    if (currentArm.armMovementEnds()) {
      publishPositions();
    }
  }
  
  
  private double[] getTargetPoint() {
    return targetPoint;
  }

  /**
   * This static method returns all the details of the class without it having
   * to be constructed. It has description, categories, dependencies, and peer
   * definitions.
   * 
   * @return ServiceType - returns all the data
   * 
   */
  static public ServiceType getMetaData() {

    ServiceType meta = new ServiceType(InverseKinematics3D.class.getCanonicalName());
    meta.addDescription("a 3D kinematics service supporting D-H parameters");
    meta.addCategory("robot", "control");

    return meta;
  }

}